% This funcion solves the mpc problem
% Input:
%    x_0        [4,1]
%    x_ref      [4*60,1]
%    x_pred     [4*60,1]
%    u_pred     [2*60,1]
%    vx         [60,1]
%    params:    weights and constraints
% Output:
%    x_pred     [4*60,1]: Predicted states for the optimal inputs
%    u_opt      [2*60,1]: Optimal inputs
%    x_comp     [4*60,1]: Predicted states for the previous (or compare) inputs

function [x_pred, u_opt, x_comp]= mpc(x_0, x_ref, x_prev, u_prev, vx, params)

% Parameters of the MPC
n_horizon = 60;
n_states = 6;
n_inputs = 2;

% Discrete model matrices for each step
Ad = cell(n_horizon,1);
Bd = cell(n_horizon,1);
Cd = cell(n_horizon,1);
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
    
    % Select model for MPC
    [Ad{i}, Bd{i}, Cd{i}] = direct_anfis_matrix(xi, ui, vxi);
    %[Ad{i}, Bd{i}, Cd{i}] = ltv_tv_matrix(xi, ui, vxi);
    %[Ad{i}, Bd{i}, Cd{i}] = ltv_matrix(xi, ui, vxi);

    % Diference between linear models
    % [A_lin, B_lin, C_lin] = ltv_tv_matrix(xi, x_ref(from_x:to_x), vxi);
    % fprintf('step %d\n', i);
    % fprintf('||A_anfis - A_lin|| = %.3e\n', norm(Ad{i} - A_lin));
    % fprintf('||B_anfis - B_lin|| = %.3e\n', norm(Bd{i} - B_lin));
    % fprintf('||C_anfis - C_lin|| = %.3e\n', norm(Cd{i} - C_lin));
    
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
    params.q_st/params.scale_st^2;
    params.q_dst/params.scale_dst^2;
]);

% Initialize P matrix
P = diag([
    params.p_y/params.scale_y^2
    params.p_vy/params.scale_vy^2
    params.p_psi/params.scale_psi^2
    params.p_r/params.scale_r^2
    params.p_st/params.scale_st^2
    params.p_dst/params.scale_dst^2;
]);

% Initialize R matrix
R = diag([
    params.r_st/params.scale_st^2
    params.r_mz/params.scale_mz^2
]);

% Create Q_ matrix
Q_ = blkdiag(kron(eye(n_horizon-1), Q), P);

% Create R_ matrix
R_ = kron(eye(n_horizon), R);

% Construct Q*S
QS = Q_ * S;

% H matrix
H = 2 * (S' * QS + R_);
H = (H + H')/2; % Ensure symetry
H = H + 1e-8*eye(size(H));
if any(~isfinite(H(:)))
    error('H contains NaN or Inf');
end

% g vector
g = 2 * S' * Q_ * (T * x_0 + W - x_ref);
if any(~isfinite(g(:)))
    error('g contains NaN or Inf');
end



% ----------------- Simple solution ---------------------
% u_opt = -H \ g;

% -------------- Optimization without restrictions ------------
% % Calculate the result
% [L, D, P] = ldl(H);
% 
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
    lb(n_inputs*(i-1)+1:n_inputs*i) = [params.min_st; params.min_mz];
    ub(n_inputs*(i-1)+1:n_inputs*i) = [params.max_st; params.max_mz];
end

% Call quadprog
options = optimoptions('quadprog', 'Display', 'off');

[u_opt, fval, exitflag] = quadprog(H, g, [], [], [], [], lb, ub, [], options);

% Check optimization status
if exitflag ~= 1
    warning('quadprog did not converge. Exitflag: %d', exitflag);
end

% Prediction
x_pred = S * u_opt + T * x_0 + W;

% Prediction to compare model
x_comp = S * u_prev + T * x_0 + W;


end