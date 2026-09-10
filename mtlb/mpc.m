% This funcion solves the mpc problem
% Input:
%    x_0        [6,1]
%    x_ref      [6*60,1]
%    x_pred     [6*60,1]
%    u_pred     [60,1]
%    vx         [60,1]
%    params:    horizon, sample time, model, weights, and constraints
% Output:
%    x_pred:    Predicted states for the optimal inputs
%    u_opt:     Optimal steering commands
%    x_comp:    Predicted states for the previous inputs
%    info:      Solver exit flag and fallback status

function [x_pred,u_opt,x_comp,info] = mpc( ...
        x_0,x_ref,x_prev,u_prev,vx,u_prev_iter,params)

total_timer = tic;

% Parameters of the MPC
n_states = 6;
n_inputs = 1;
n_horizon = params.n_horizon;
dt = params.Ts;

validateattributes(n_horizon,{'numeric'}, ...
    {'scalar','integer','positive'},mfilename,'params.n_horizon');
validateattributes(dt,{'numeric'}, ...
    {'scalar','real','finite','positive'},mfilename,'params.Ts');
validateattributes(x_0,{'numeric'}, ...
    {'vector','numel',n_states,'finite'},mfilename,'x_0');
validateattributes(x_ref,{'numeric'}, ...
    {'vector','numel',n_states*n_horizon,'finite'},mfilename,'x_ref');
validateattributes(x_prev,{'numeric'}, ...
    {'vector','numel',n_states*n_horizon,'finite'},mfilename,'x_prev');
validateattributes(u_prev,{'numeric'}, ...
    {'vector','numel',n_inputs*n_horizon,'finite'},mfilename,'u_prev');
validateattributes(vx,{'numeric'}, ...
    {'vector','numel',n_horizon,'finite'},mfilename,'vx');
validateattributes(u_prev_iter,{'numeric'}, ...
    {'vector','nonempty','finite'},mfilename,'u_prev_iter');

x_0 = x_0(:);
x_ref = x_ref(:);
x_prev = x_prev(:);
u_prev = u_prev(:);
vx = vx(:);

% Discrete model matrices for each step
Ad = cell(n_horizon,1);
Bd = cell(n_horizon,1);
Cd = cell(n_horizon,1);
model_matrix_timer = tic;
for i = 1:n_horizon
    from_u = (i-1)*n_inputs + 1;
    to_u   = i*n_inputs;
    ui = u_prev(from_u:to_u);

    if i == 1
        xi = x_0;
    else
        from_x = (i-2)*n_states + 1;
        to_x   = (i-1)*n_states;
        xi = x_prev(from_x:to_x);
    end

    vxi = vx(i);
    
    % Instantiate the configured model about the shifted previous solution.
    [Ad{i}, Bd{i}, Cd{i}] = configuredModelMatrix( ...
        params.model,xi,ui,vxi,dt);

    % Check inputs and matrixes
    assert(all(isfinite(xi)), 'x_prev invalid at step %d', i);
    assert(all(isfinite(ui)), 'u_prev invalid at step %d', i);
    assert(isfinite(vxi), 'vx invalid at step %d', i);
    assert(all(isfinite(Ad{i}(:))), 'Ad invalid at step %d', i);
    assert(all(isfinite(Bd{i}(:))), 'Bd invalid at step %d', i);
    assert(all(isfinite(Cd{i}(:))), 'Cd invalid at step %d', i);

    if norm(Ad{i}, inf) > 1e3
        warning('Large Ad at step %d: norm=%g', i, norm(Ad{i}, inf));
    end
    if norm(Bd{i}, inf) > 1e3
        warning('Large Bd at step %d: norm=%g', i, norm(Bd{i}, inf));
    end
end
info.model_matrix_time = toc(model_matrix_timer);

% Fill matrix T
T = zeros(n_states * n_horizon, n_states);
for i = 1:n_horizon
    rows = (n_states * (i - 1) + 1):(n_states * i);
    Aprod = eye(n_states);
    for k = 1:i
        Aprod = Ad{k} * Aprod;
    end
    T(rows, 1:n_states) = Aprod;
end

% Fill matrix S
S = zeros(n_states * n_horizon, n_horizon * n_inputs);
for i = 1:n_horizon
    for j = 1:i
        Aprod = eye(n_states);
        for k = (j+1):i
            Aprod = Ad{k} * Aprod;
        end
        rows = (n_states * (i - 1) + 1):(n_states * i);
        cols = (n_inputs * (j - 1) + 1):(n_inputs * j);
        S(rows,cols) = Aprod * Bd{j};
    end
end

% Fill matrix W
W = zeros(n_states * n_horizon, 1);
for i = 1:n_horizon
    rows = (n_states*(i-1)+1):(n_states*i);
    wk = zeros(n_states, 1);

    for j = 1:i
        Aprod = eye(n_states);
        for k = j+1:i
            Aprod = Ad{k} * Aprod;
        end
        wk = wk + Aprod * Cd{j};
    end

    W(rows,1) = wk;
end

% Initialize Q matrix
Q = diag([
    params.q_y/params.scale_y^2
    params.q_vy/params.scale_vy^2
    params.q_psi/params.scale_psi^2
    params.q_r/params.scale_r^2
    params.q_st/params.scale_st^2
    params.q_dst/params.scale_dst^2
]);

% Initialize P matrix
P = diag([
    params.p_y/params.scale_y^2
    params.p_vy/params.scale_vy^2
    params.p_psi/params.scale_psi^2
    params.p_r/params.scale_r^2
    params.p_st/params.scale_st^2
    params.p_dst/params.scale_dst^2
]);

% Initialize R matrix
R = params.r_st/params.scale_st^2;

% Initialize Rd matrix
Rd = params.rd_st/params.scale_dst^2;

% Create Q_ matrix
Q_ = blkdiag(kron(eye(n_horizon-1), Q), P);

% Create R_ matrix
R_ = kron(eye(n_horizon), R);

% Create Rd_ matrix
Rd_ = kron(eye(n_horizon), Rd);

% Create D
D = zeros(n_horizon*n_inputs, n_horizon*n_inputs);
I = eye(n_inputs);
for k = 1:n_horizon
    % Diagonal block
    rows = (k-1)*n_inputs + (1:n_inputs);
    cols = (k-1)*n_inputs + (1:n_inputs);
    D(rows, cols) = I;

    % Subdiagonal block
    if k > 1
        cols_prev = (k-2)*n_inputs + (1:n_inputs);
        D(rows, cols_prev) = -I;
    end
end

% Create d
d = zeros(n_horizon*n_inputs, 1);
d(1:n_inputs) = -u_prev_iter(1:n_inputs);

% H matrix
H = 2 * (S' * Q_ * S + R_ + D' * Rd_ * D);
H = (H + H')/2; % Ensure symetry
H = H + 1e-8*eye(size(H));
if any(~isfinite(H(:)))
    error('H contains NaN or Inf');
end

% g vector
g = 2 * S' * Q_ * (T * x_0 + W - x_ref) + 2 * D' * Rd_ * d;
if any(~isfinite(g(:)))
    error('g contains NaN or Inf');
end



% ----------------- Simple solution ---------------------
% u_opt = -H \ g;

% -------------- Optimization without restrictions ------------
% % Calculate the result
% [L, D, P] = ldl(H);

% % Solve the system H * delta_u_opt = -g
% rhs = P * (-g);   % permuted right-hand side
% z   = L \ rhs;    % forward solve
% w   = D \ z;      % diagonal / block-diagonal solve
% ut  = L' \ w;     % backward solve
% u_opt = P' * ut;  % unpermute

% -------------- Optimization with restrictions ------------
% Define bounds
lb = zeros(n_horizon*n_inputs, 1);
ub = zeros(n_horizon*n_inputs, 1);
for i = 1:n_horizon
    lb(n_inputs*(i-1)+1:n_inputs*i) = params.min_st;
    ub(n_inputs*(i-1)+1:n_inputs*i) = params.max_st;
end

solve_problem = ~isfield(params,'solve_problem') || params.solve_problem;
if solve_problem
    % Hard limits on command increments and predicted actuator states. The
    % command increment is converted from rad/s to rad/sample using Ts.
    free_prediction = T*x_0 + W;
    max_command_step = params.max_st_rate*dt;
    Aineq = [D;-D];
    bineq = [max_command_step*ones(n_horizon,1)-d; ...
             max_command_step*ones(n_horizon,1)+d];

    select_delta = kron(eye(n_horizon),[0 0 0 0 1 0]);
    select_delta_dot = kron(eye(n_horizon),[0 0 0 0 0 1]);
    S_delta = select_delta*S;
    S_delta_dot = select_delta_dot*S;
    free_delta = select_delta*free_prediction;
    free_delta_dot = select_delta_dot*free_prediction;
    Aineq = [Aineq;S_delta;-S_delta;S_delta_dot;-S_delta_dot];
    bineq = [bineq; ...
        params.max_delta*ones(n_horizon,1)-free_delta; ...
        params.max_delta*ones(n_horizon,1)+free_delta; ...
        params.max_st_rate*ones(n_horizon,1)-free_delta_dot; ...
        params.max_st_rate*ones(n_horizon,1)+free_delta_dot];

    options = optimoptions('quadprog','Display','off');
    solver_timer = tic;
    [u_opt,~,exitflag] = quadprog( ...
        H,g,Aineq,bineq,[],[],lb,ub,[],options);
    info.solver_time = toc(solver_timer);
    info.exitflag = exitflag;
    info.used_fallback = exitflag ~= 1;

    if exitflag ~= 1
        warning('quadprog did not converge. Exitflag: %d',exitflag);
        u_opt = u_prev;
    end
else
    % Prediction-only mode is used by the model-error diagnostic. It must
    % not solve an unrelated constrained tracking problem.
    u_opt = u_prev;
    info.exitflag = NaN;
    info.used_fallback = false;
    info.solver_time = 0;
end

% Prediction
x_pred = S * u_opt + T * x_0 + W;

% Prediction to compare model
x_comp = S * u_prev + T * x_0 + W;
info.total_time = toc(total_timer);

end

function [A,B,C] = configuredModelMatrix(model_name,x,u,vx,dt)
switch lower(string(model_name))
    case "ltv"
        [A,B,C] = ltv_matrix(x,u,vx,dt);
    case "anfis_direct"
        [A,B,C] = anfis_direct_matrix(x,u,vx,dt);
    case "anfis_delta"
        [A,B,C] = anfis_delta_matrix(x,u,vx,dt);
    case "anfis_derivative"
        [A,B,C] = anfis_dot_matrix(x,u,vx,dt);
    case "anfis_ltv_residual"
        [A,B,C] = anfis_residuals_matrix(x,u,vx,dt);
    case "eefig"
        [A,B,C] = eefig_matrix(x,u,vx,dt);
    otherwise
        error('Unknown MPC model "%s".',string(model_name));
end
end
