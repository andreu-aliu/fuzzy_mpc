% This funcion solves the mpc problem
% Input:
%    x_0        [4,1]
%    x_ref      [4*60,1]
%    x_pred     [4*60,1]
%    u_pred     [2*60,1]
%    vx         [60,1]
%    params: weights and constraints
% Output:
%    u_opt      [2*60,1]

function [x_pred, u_opt]= mpc(x_0, x_ref, x_prev, u_prev, vx, params)

% Parameters of the MPC
n_horizon = 60;
n_states = 4;
n_inputs = 2;

% Ciscrete model matrices for each step
Ad = cell(n_horizon,1);
Bd = cell(n_horizon,1);
Cd = cell(n_horizon,1);
for i = 1:n_horizon
    from_x = i * n_states - n_states + 1;
    to_x = i * n_states;
    from_u = i * n_inputs - n_inputs + 1;
    to_u = i * n_inputs;
    [Ad{i}, Bd{i}, Cd{i}] = direct_anfis_matrix(x_prev(from_x:to_x), u_prev(from_u:to_u), vx(i));
    %[Ad{i}, Bd{i}, Cd{i}] = ltv_tv_matrix(x_prev(from_x:to_x), x_ref(from_u:to_u), vx(i));
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

% Scales for normalization
scale_y   = 1.0;   % m
scale_vy  = 0.1;   % m/s
scale_psi = 0.05;  % rad
scale_r   = 0.5;   % rad/s
scale_st  = 0.2;   % rad
scale_mz  = 1000;  % Nm

% Initialize Q matrix
Q = diag([
    params.q_y/scale_y^2
    params.q_vy/scale_vy^2
    params.q_psi/scale_psi^2
    params.q_r/scale_r^2
]);

% Initialize P matrix
P = diag([
    parmas.p_y/scale_y^2
    params.p_vy/scale_vy^2
    params.p_psi/scale_psi^2
    params.p_r/scale_r^2
]);

% Initialize R matrix
R = diag([
    params.r_st/scale_st^2
    params.r_mz/scale_mz^2
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

% g vector
g = 2 * S' * Q_ * (T * x_0 + W - x_ref);

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


end