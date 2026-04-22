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

% S
n_rows = 6 * n_horizon;
n_cols = 2 * n_horizon;
data = readmatrix("../debug/S.csv");
row_data = data(iter,:);
S_mat = reshape(row_data, n_cols, n_rows).';

% T
n_rows = 6 * n_horizon;
n_cols = 1;
data = readmatrix("../debug/W.csv");
row_data = data(iter,:);
W_mat = reshape(row_data, n_cols, n_rows).';

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

% H
n_rows = 2 * n_horizon;
n_cols = 2 * n_horizon;
data = readmatrix("../debug/H.csv");
row_data = data(iter,:);
H_mat = reshape(row_data, n_cols, n_rows).';

% g
n_rows = 2 * n_horizon;
n_cols = 1;
data = readmatrix("../debug/g.csv");
row_data = data(iter,:);
g_mat = reshape(row_data, n_cols, n_rows).';

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