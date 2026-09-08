% This script converts saved csv matrices from C++ MPC to Matlab variables to debug
cd("/home/andreu/ros_ws/src/as/control/fuzzy_mpc/mtlb")
iter = 1;
n_horizon = 60;

% --------   Input   ------------
% x0
n_rows = 6;
n_cols = 1;
data = readmatrix("../debug/x0.csv");
fprintf("Elements in x0: %i \n", size(data,2))
row_data = data(iter,:);
x_0 = reshape(row_data, n_cols, n_rows).';

% x_ref
n_rows = 6 * n_horizon;
n_cols = 1;
data = readmatrix("../debug/x_ref.csv");
row_data = data(iter,:);
x_ref = reshape(row_data, n_cols, n_rows).';

% vx
n_rows = n_horizon;
n_cols = 1;
data = readmatrix("../debug/vx.csv");
row_data = data(iter,1:n_rows);
vx = reshape(row_data, n_cols, n_rows).';

% x_prev
n_rows = 6 * n_horizon;
n_cols = 1;
data = readmatrix("../debug/x_prev.csv");
row_data = data(iter,:);
x_prev = reshape(row_data, n_rows, n_cols).';

% u_prev
n_rows = 2 * n_horizon;
n_cols = 1;
data = readmatrix("../debug/u_prev.csv");
row_data = data(iter,:);
u_prev = reshape(row_data, n_cols, n_rows).';

% --------   States   ------------
% T
n_rows = 6 * n_horizon;
n_cols = 6;
data = readmatrix("../debug/T.csv");
row_data = data(iter,:);
T_mat = reshape(row_data, n_cols, n_rows).';
params.T_mat = T_mat;

% S
n_rows = 6 * n_horizon;
n_cols = 2 * n_horizon;
data = readmatrix("../debug/S.csv");
row_data = data(iter,:);
S_mat = reshape(row_data, n_cols, n_rows).';
params.S_mat = S_mat;

% T
n_rows = 6 * n_horizon;
n_cols = 1;
data = readmatrix("../debug/W.csv");
row_data = data(iter,:);
W_mat = reshape(row_data, n_cols, n_rows).';
params.T_mat = T_mat;

% --------   Weights matrices   ------------
% R_
n_rows = 2 * n_horizon;
n_cols = 2 * n_horizon;
data = readmatrix("../debug/R_.csv");
row_data = data(iter,:);
R_mat = reshape(row_data, n_cols, n_rows).';

% Rd_
n_rows = 2 * n_horizon;
n_cols = 2 * n_horizon;
data = readmatrix("../debug/Rd_.csv");
row_data = data(iter,:);
Rd_mat = reshape(row_data, n_cols, n_rows).';

% D
n_rows = 2 * n_horizon;
n_cols = 2 * n_horizon;
data = readmatrix("../debug/D_.csv");
row_data = data(iter,:);
D_mat = reshape(row_data, n_cols, n_rows).';

% Q
data = readmatrix("../debug/q_diag.csv");
row_data = data(iter,:);
Q_mat = diag(row_data);


% --------   Solving matrices   ------------
% d
n_rows = 2 * n_horizon;
n_cols = 1;
data = readmatrix("../debug/d.csv");
row_data = data(iter,:);
d_mat = reshape(row_data, n_cols, n_rows).';
params.d_mat = d_mat;

% H
n_rows = 2 * n_horizon;
n_cols = 2 * n_horizon;
data = readmatrix("../debug/H.csv");
row_data = data(iter,:);
H_mat = reshape(row_data, n_cols, n_rows).';
param.H_mat = H_mat;

% g
n_rows = 2 * n_horizon;
n_cols = 1;
data = readmatrix("../debug/g.csv");
row_data = data(iter,:);
g_mat = reshape(row_data, n_cols, n_rows).';
params.g_mat = g_mat;

% u_opt
n_rows = 2 * n_horizon;
n_cols = 1;
data = readmatrix("../debug/u_opt.csv");
row_data = data(iter,:);
u_opt_mat = reshape(row_data, n_cols, n_rows).';


% --------   Prediction   ------------
% x_pred
n_rows = 6 * n_horizon;
n_cols = 1;
data = readmatrix("../debug/x_pred.csv");
row_data = data(iter,:);
x_pred_mat = reshape(row_data, n_cols, n_rows).';

%% MPC PARAMETERS & OPTIONS

Np = 60;
nx = 6;
nu = 1;
dt = 0.02;

% Scales for normalization
params.scale_y   = 0.1;   % m
params.scale_vy  = 0.1;   % m/s
params.scale_psi = 0.05;  % rad
params.scale_r   = 0.05;   % rad/s
params.scale_st  = 0.2;   % rad
params.scale_dst = 0.001;   % rad/0.02s

% Weights
params.q_y  = 200;
params.q_vy = 0;
params.q_psi= 0;
params.q_r  = 1;
params.q_st = 0;
params.q_dst= 0;

params.p_y  = 1000;
params.p_vy = 0;
params.p_psi= 0;
params.p_r  = 0;
params.p_st = 0;
params.p_dst= 0;

params.r_st = 1;

params.rd_st = 2;

% Bounds
params.min_st = -0.38; % 436
params.max_st = 0.38;
%% Compute MPC with given matrices

[x_pred, u_opt, x_comp] = mpc(x_0, x_ref, x_prev, u_prev, vx, u_prev, params);
