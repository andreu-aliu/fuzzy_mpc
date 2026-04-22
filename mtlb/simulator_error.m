% Simulate with same inputs the and compare with MPC model
clear all
Np = 60;
dt = 0.02;

%% MP parameters
% Scales for normalization
params.scale_y   = 0.3;   % m
params.scale_vy  = 0.1;   % m/s
params.scale_psi = 0.05;  % rad
params.scale_r   = 0.5;   % rad/s
params.scale_st  = 0.2;   % rad
params.scale_dst = 1.0;   % rad/s
params.scale_mz  = 1000;  % Nm

% Weights
params.q_y  = 100;
params.q_vy = 0;
params.q_psi= 4;
params.q_r  = 0;
params.q_st = 0;
params.q_dst= 3;

params.p_y  = 500;
params.p_vy = 0;
params.p_psi= 5;
params.p_r  = 0;
params.p_st = 0;
params.p_dst= 0;

params.r_st = 10;
params.r_mz = 0; 

params.rd_st = 2;
params.rd_mz = 0; 

% Bounds
params.min_st = -0.436;
params.max_st = 0.436;
params.min_mz = -0;
params.max_mz = 0;
%%
% Generate first global state [x, y, psi, vx, vy, r, delta, delta_dot]
vx = 020;
X_0 = [10 10 0.01 vx 0 0 0 0];

% Generate inputs as sinusoidal signals
t = (1:60) * dt;
u_mz = sin(t*2*pi)* 100;
u_st = sin(t*2*pi)* 0.4;
U = mat2cell([u_st' u_mz'], ones(Np,1), 2);

% Iterate over states
X_sim = cell(Np,1); X_mat = cell(Np,1);
X_sim{1} = X_0;     X_mat{1} = [X_0(2), X_0(5), X_0(3), X_0(6), X_0(7), X_0(8)];
Ad = cell(Np,1); Bd = cell(Np,1); Cd = cell(Np,1);
for i = 1:Np

    % Output from simulator
    X_sim{i+1} = sim_anfis_delta(X_sim{i}, U{i}, vx, dt);

    % Model matrices
    [Ad{i}, Bd{i}, Cd{i}] = anfis_delta_matrix(X_mat{i}, U{i}', X_sim{i}(4));
    % Output from matrices
    X_mat{i+1} = (Ad{i} * X_mat{i}' + Bd{i} * U{i}' + Cd{i})';
end

% Extract matrices
from_sim = cell2mat(X_sim);
from_mat = cell2mat(X_mat);

% MPC solution

    % Convert to local
    x_0_comp = X_0([2 5 3 6 7 8])';
    x_comp = from_sim(2:end, [2 5 3 6 7 8]);
    u_comp = cell2mat(U);
    vx_comp = from_sim(2:end, 4);

    % Use MPC function again to compute the predicted with 
    x_comp_vec  = reshape(x_comp.', [], 1);
    u_comp_vec  = reshape(u_comp.', [], 1);

    [~,~,x_pred_comp_vec] = mpc(x_0_comp, x_comp_vec, x_comp_vec, u_comp_vec, vx_comp, zeros(2*Np,1), params);

    x_pred_comp = reshape([x_0_comp ;x_pred_comp_vec], 6, []).';


% Time vector (Np+1 because of propagation)
t_full = (0:Np) * dt;

% Simulator signals
y_sim   = from_sim(:, 2);
vy_sim  = from_sim(:, 5);
psi_sim = from_sim(:, 3);
r_sim   = from_sim(:, 6);
vx_sim  = from_sim(:, 4);
delta_sim = from_sim(:, 7);

% Matrix model signals
y_mat   = from_mat(:, 1);
vy_mat  = from_mat(:, 2);
psi_mat = from_mat(:, 3);
r_mat   = from_mat(:, 4);
delta_mat = from_mat(:, 5);

% MPC signals
y_mpc   = x_pred_comp(:, 1);
vy_mpc  = x_pred_comp(:, 2);
psi_mpc = x_pred_comp(:, 3);
r_mpc   = x_pred_comp(:, 4);
delta_mpc = x_pred_comp(:, 5);


% Plots
figure()
tiledlayout(6,1)

% Y
nexttile
plot(t_full, y_sim, 'b', t_full, y_mat, 'r', t_full, y_mpc, 'g')
title('y')
legend('sim','mat','mpc')
grid on

% vy
nexttile
plot(t_full, vy_sim, 'b', t_full, vy_mat, 'r', t_full, vy_mpc, 'g')
title('vy')
legend('sim','mat','mpc')
grid on

% psi
nexttile
plot(t_full, psi_sim, 'b', t_full, psi_mat, 'r', t_full, psi_mpc, 'g')
title('psi')
legend('sim','mat','mpc')
grid on

% r
nexttile
plot(t_full, r_sim, 'b', t_full, r_mat, 'r', t_full, r_mpc, 'g')
title('r')
legend('sim','mat','mpc')
grid on

% vx (only simulator, since model doesn't propagate it)
nexttile
plot(t_full, vx_sim, 'b')
title('vx (sim only)')
grid on

% Delta
nexttile
plot(t_full, delta_sim, 'b', t_full, delta_mat, 'r', t_full, delta_mpc, 'g')
title('Delta')
legend('sim','mat','mpc')
grid on
