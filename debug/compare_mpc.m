% This script is used to compare ltv_mpc matrices to fuzzy_mpc for the same vehicle model

%% Load all csv from the MPC results

idx = 2;

% LTV_MPC
ltv_path = "/home/andreu/ros_ws/src/as/control/ltv_mpc/test/data";
ltv.x0 = csv2mat(ltv_path + "/x0.csv", idx, 1, 6);
ltv.x_ref = csv2mat(ltv_path + "/x_ref.csv", idx, 6, 60)';
ltv.transformed_headings = csv2mat(ltv_path + "/transformed_headings.csv", idx, 1, 61)';
ltv.prev_psi = ltv.transformed_headings(2:end);
ltv.prev_delta = csv2mat(ltv_path + "/prev_delta.csv", idx, 1, 60)';
ltv.T = csv2mat(ltv_path + "/T.csv", idx, 6, 60*6)';
ltv.S = csv2mat(ltv_path + "/S.csv", idx, 60*1, 60*6)';
ltv.q_diag = csv2mat(ltv_path + "/q_diag.csv", 2, 60*6, 1)';
ltv.r = csv2mat(ltv_path + "/R_.csv", 2, 60,60)';
ltv.x_pred = csv2mat(ltv_path + "/x_pred", idx, 6,60)';
ltv.u_opt = csv2mat(ltv_path + "/u_opt", idx, 1, 60)';
ltv.vx = csv2mat(ltv_path + "/vx", idx, 1, 61)';
ltv.H = csv2mat(ltv_path + "/H.csv", idx, 60,60)';
ltv.g = csv2mat(ltv_path + "/g.csv", idx, 1, 60)';

% Fuzzy_MPC
fuz_path = "/home/andreu/ros_ws/src/as/control/fuzzy_mpc/debug/csv";
fuz.x0 = csv2mat(fuz_path + "/x0.csv", idx, 1, 6);
fuz.x_ref = csv2mat(fuz_path + "/x_ref.csv", idx, 6, 60)';
fuz.x_prev = csv2mat(fuz_path + "/x_prev.csv", idx, 6, 60)';
fuz.T = csv2mat(fuz_path + "/T.csv", idx, 6, 60*6)';
fuz.S_all = csv2mat(fuz_path + "/S.csv", idx, 60*2, 60*6)';
fuz.S = fuz.S_all(:, 1:2:end);
fuz.q_diag = csv2mat(fuz_path + "/q_diag.csv", 1, 60*6, 1)';
fuz.r_all = csv2mat(fuz_path + "/R_.csv", 1, 2*60,2*60);
fuz.r = fuz.r_all(1:2:end,1:2:end);
fuz.x_pred = csv2mat(fuz_path + "/x_pred", idx, 6,60)';
fuz.u_opt_all = csv2mat(fuz_path + "/u_opt", idx, 2, 60)';
fuz.u_opt = fuz.u_opt_all(:, 1);
fuz.vx = csv2mat(fuz_path + "/vx.csv", idx, 1, 60);
fuz.H_all = csv2mat(fuz_path + "/H.csv", idx, 60*2,60*2)';
fuz.H = fuz.H_all(1:2:end,1:2:end);
fuz.g_all = csv2mat(fuz_path + "/g.csv", idx, 1, 60*2)';
fuz.g = fuz.g_all(1:2:end,:);


%% 1. Plot references and previous states
figure(1); clf; set(gcf, 'Name', 'References and starting point');
tiledlayout(7,1);

% y
nexttile();
plot(ltv.x_ref(:,1), 'rx-'); hold on;
plot(ltv.x0(1), 'r.', 'MarkerSize', 15)
plot(fuz.x_ref(:,1), 'bx-');
plot(fuz.x0(1), 'b.', 'MarkerSize', 15)
title("y")

% vy
nexttile()
plot(ltv.x_ref(:,2), 'rx-'); hold on;
plot(ltv.x0(2), 'r.', 'MarkerSize', 15)
plot(fuz.x_ref(:,2), 'bx-');
plot(fuz.x0(2), 'b.', 'MarkerSize', 15)
title("vy")

% psi
nexttile()
plot(ltv.x_ref(:,3), 'rx-'); hold on;
plot(ltv.x0(3), 'r.', 'MarkerSize', 15)
plot(fuz.x_ref(:,3), 'bx-');
plot(fuz.x0(3), 'b.', 'MarkerSize', 15)
title("psi")

% r
nexttile()
plot(ltv.x_ref(:,4), 'rx-'); hold on;
plot(ltv.x0(4), 'r.', 'MarkerSize', 15)
plot(fuz.x_ref(:,4), 'bx-');
plot(fuz.x0(4), 'b.', 'MarkerSize', 15)
title("r")

% delta
nexttile()
plot(ltv.x_ref(:,5), 'rx-'); hold on;
plot(ltv.x0(5), 'r.', 'MarkerSize', 15)
plot(fuz.x_ref(:,5), 'bx-');
plot(fuz.x0(5), 'b.', 'MarkerSize', 15)
title("delta")

% delta_dot
nexttile()
plot(ltv.x_ref(:,6), 'rx-'); hold on;
plot(ltv.x0(6), 'r.', 'MarkerSize', 15)
plot(fuz.x_ref(:,6), 'bx-');
plot(fuz.x0(6), 'b.', 'MarkerSize', 15)
title("delta dot")

legend("ltv ref", "ltv x0", "fuzzy ref", "fuzzy x0")

% vx
nexttile()
plot(ltv.vx, 'rx-'); hold on;
plot(fuz.vx, 'bx-');
title("vx")



figure(2); clf; set(gcf, 'Name', 'Previous states');
tiledlayout(6,1);

% y
nexttile();
plot(fuz.x_prev(:,1), 'bx-');
title("y")

% vy
nexttile()
plot(fuz.x_prev(:,2), 'bx-');
title("vy")

% psi
nexttile()
plot(ltv.prev_psi(:,1), 'r'); hold on;
plot(fuz.x_prev(:,3), 'bx-');
title("psi")

% r
nexttile()
plot(fuz.x_prev(:,4), 'bx-');
title("r")

% delta
nexttile()
plot(ltv.prev_delta(:,1), 'r'); hold on;
plot(fuz.x_prev(:,5), 'bx-');
title("delta")
legend("ltv prev", "fuzzy prev")

% delta_dot
nexttile()
plot(fuz.x_prev(:,6), 'bx-');
title("delta dot")



%% 2. Compare T and S matrices

nx = 6;
state_names = ["y", "vy", "psi", "r", "delta", "delta_dot"];

% ===================== T =====================

E_T = ltv.T - fuz.T;

T_abs_fro = norm(E_T, 'fro');
T_rel_fro = T_abs_fro / max(norm(ltv.T, 'fro'), 1e-12);
T_max_abs = max(abs(E_T), [], 'all');

[max_T, idx_T] = max(abs(E_T(:)));
[row_T, col_T] = ind2sub(size(E_T), idx_T);

pred_step_T = floor((row_T - 1) / nx) + 1;
state_T = mod(row_T - 1, nx) + 1;
x0_col_T = col_T;

fprintf("\n========== T ==========\n");
fprintf("T abs fro: %.6e\n", T_abs_fro);
fprintf("T rel fro: %.6e\n", T_rel_fro);
fprintf("T max abs: %.6e\n", T_max_abs);

fprintf("\nWorst T error:\n");
fprintf("  row, col  = (%d, %d)\n", row_T, col_T);
fprintf("  pred step = %d\n", pred_step_T);
fprintf("  state     = %d (%s)\n", state_T, state_names(state_T));
fprintf("  x0 col    = %d", x0_col_T);
if x0_col_T <= nx
    fprintf(" (%s)", state_names(x0_col_T));
end
fprintf("\n");
fprintf("  ltv.T     = %.17g\n", ltv.T(row_T, col_T));
fprintf("  fuz.T     = %.17g\n", fuz.T(row_T, col_T));
fprintf("  diff      = %.17g\n", E_T(row_T, col_T));

% Plot T

all_vals = [ltv.T(:); fuz.T(:)];
clim_main = [min(all_vals), max(all_vals)];

dmax = max(abs(E_T), [], 'all');
clim_diff = [-dmax, dmax];

figure(3); set(gcf, "Name", "T matrix comparison");

subplot(1,3,1)
imagesc(ltv.T);
clim(clim_main);
colorbar;
title("LTV T");

subplot(1,3,2)
imagesc(fuz.T);
clim(clim_main);
colorbar;
title("Fuzzy T");

subplot(1,3,3)
imagesc(E_T);
clim(clim_diff);
colorbar;
title("T Difference");

% T block error per prediction step

N_T = size(ltv.T, 1) / nx;
T_block_err = zeros(N_T, 1);

for k = 1:N_T
    rows = (k - 1) * nx + (1:nx);
    T_block_err(k) = norm(E_T(rows, :), 'fro');
end

figure(4); set(gcf, "Name", "T block error");
plot(1:N_T, T_block_err, '-o');
grid on;
xlabel("Prediction step");
ylabel("||T block error||_fro");
title("T error per prediction step");


% ===================== S =====================

E_S = ltv.S - fuz.S;

S_abs_fro = norm(E_S, 'fro');
S_rel_fro = S_abs_fro / max(norm(ltv.S, 'fro'), 1e-12);
S_max_abs = max(abs(E_S), [], 'all');

[max_S, idx_S] = max(abs(E_S(:)));
[row_S, col_S] = ind2sub(size(E_S), idx_S);

pred_step_S = floor((row_S - 1) / nx) + 1;
state_S = mod(row_S - 1, nx) + 1;
control_step_S = col_S;

fprintf("\n========== S ==========\n");
fprintf("S abs fro: %.6e\n", S_abs_fro);
fprintf("S rel fro: %.6e\n", S_rel_fro);
fprintf("S max abs: %.6e\n", S_max_abs);

fprintf("\nWorst S error:\n");
fprintf("  row, col     = (%d, %d)\n", row_S, col_S);
fprintf("  pred step    = %d\n", pred_step_S);
fprintf("  state        = %d (%s)\n", state_S, state_names(state_S));
fprintf("  control step = %d\n", control_step_S);
fprintf("  ltv.S        = %.17g\n", ltv.S(row_S, col_S));
fprintf("  fuz.S        = %.17g\n", fuz.S(row_S, col_S));
fprintf("  diff         = %.17g\n", E_S(row_S, col_S));

if pred_step_S < control_step_S
    fprintf("  diagnosis    = ERROR above diagonal: S indexing problem likely.\n");
elseif pred_step_S == control_step_S
    fprintf("  diagnosis    = diagonal block: check Bd at step %d.\n", control_step_S);
else
    fprintf("  diagnosis    = propagated block: check Ad propagation from step %d to %d.\n", ...
        control_step_S, pred_step_S);
end

% Plot S

all_vals = [ltv.S(:); fuz.S(:)];
clim_main = [min(all_vals), max(all_vals)];

dmax = max(abs(E_S), [], 'all');
clim_diff = [-dmax, dmax];

figure(5); set(gcf, "Name", "S matrix comparison");

subplot(1,3,1)
imagesc(ltv.S);
clim(clim_main);
colorbar;
title("LTV S");

subplot(1,3,2)
imagesc(fuz.S);
clim(clim_main);
colorbar;
title("Fuzzy S");

subplot(1,3,3)
imagesc(E_S);
clim(clim_diff);
colorbar;
title("S Difference");

% S block error per prediction/control step

N_pred = size(ltv.S, 1) / nx;
N_ctrl = size(ltv.S, 2);

S_block_err = zeros(N_pred, N_ctrl);

for i = 1:N_pred
    rows = (i - 1) * nx + (1:nx);

    for j = 1:N_ctrl
        S_block_err(i,j) = norm(E_S(rows, j), 'fro');
    end
end

figure(6); set(gcf, "Name", "S block error");
imagesc(S_block_err);
colorbar;
xlabel("Control step");
ylabel("Prediction step");
title("S block error");


%% 3. Compare weights matrices

% q_diag
q_diff = norm(ltv.q_diag - fuz.q_diag, 'fro')

% R
r_diff = norm(ltv.r - fuz.r, 'fro')

%% 4. Compare H and g matrices and display them

% ===================== g =====================
E_g = ltv.g - fuz.g;

g_abs_norm = norm(E_g, 2);
g_rel_norm = g_abs_norm / max(norm(ltv.g, 2), 1e-12);
g_max_abs = max(abs(E_g));

[~, idx_g] = max(abs(E_g));
row_g = idx_g;

fprintf("\n========== g ==========\n");
fprintf("g abs norm: %.6e\n", g_abs_norm);
fprintf("g rel norm: %.6e\n", g_rel_norm);
fprintf("g max abs : %.6e\n", g_max_abs);

fprintf("\nWorst g error:\n");
fprintf("  index     = %d\n", row_g);
fprintf("  ltv.g     = %.17g\n", ltv.g(row_g));
fprintf("  fuz.g     = %.17g\n", fuz.g(row_g));
fprintf("  diff      = %.17g\n", E_g(row_g));

figure(7); set(gcf, "Name", "g comparison");

subplot(1,3,1)
plot(ltv.g, '-o'); grid on;
title("LTV g");

subplot(1,3,2)
plot(fuz.g, '-o'); grid on;
title("Fuzzy g");

subplot(1,3,3)
plot(E_g, '-o'); grid on;
title("g difference");

nx = 6;

pred_step_g = floor((row_g - 1) / nx) + 1;
state_g = mod(row_g - 1, nx) + 1;

fprintf("  pred step = %d\n", pred_step_g);
fprintf("  state     = %d (%s)\n", state_g, state_names(state_g));


% ===================== H =====================
E_H = ltv.H - fuz.H;

H_abs_fro = norm(E_H, 'fro');
H_rel_fro = H_abs_fro / max(norm(ltv.H, 'fro'), 1e-12);
H_max_abs = max(abs(E_H), [], 'all');

[~, idx_H] = max(abs(E_H(:)));
[row_H, col_H] = ind2sub(size(E_H), idx_H);

fprintf("\n========== H ==========\n");
fprintf("H abs fro: %.6e\n", H_abs_fro);
fprintf("H rel fro: %.6e\n", H_rel_fro);
fprintf("H max abs: %.6e\n", H_max_abs);

fprintf("\nWorst H error:\n");
fprintf("  row, col = (%d, %d)\n", row_H, col_H);
fprintf("  ltv.H    = %.17g\n", ltv.H(row_H, col_H));
fprintf("  fuz.H    = %.17g\n", fuz.H(row_H, col_H));
fprintf("  diff     = %.17g\n", E_H(row_H, col_H));

nx = 6;

pred_i = floor((row_H - 1) / nx) + 1;
state_i = mod(row_H - 1, nx) + 1;

pred_j = floor((col_H - 1) / nx) + 1;
state_j = mod(col_H - 1, nx) + 1;

fprintf("  block (i,j) = (%d, %d)\n", pred_i, pred_j);
fprintf("  states      = (%s, %s)\n", ...
    state_names(state_i), state_names(state_j));

sym_err_ltv = norm(ltv.H - ltv.H', 'fro');
sym_err_fuz = norm(fuz.H - fuz.H', 'fro');

fprintf("\nSymmetry check:\n");
fprintf("  LTV  ||H - H'||_fro = %.6e\n", sym_err_ltv);
fprintf("  Fuzzy||H - H'||_fro = %.6e\n", sym_err_fuz);

all_vals = [ltv.H(:); fuz.H(:)];
clim_main = [min(all_vals), max(all_vals)];

dmax = max(abs(E_H), [], 'all');
clim_diff = [-dmax, dmax];

figure(8); set(gcf, "Name", "H matrix comparison");

subplot(1,3,1)
imagesc(ltv.H);
clim(clim_main);
colorbar;
title("LTV H");

subplot(1,3,2)
imagesc(fuz.H);
clim(clim_main);
colorbar;
title("Fuzzy H");

subplot(1,3,3)
imagesc(E_H);
clim(clim_diff);
colorbar;
title("H Difference");

nx = 6;
N = size(ltv.H, 1) / nx;

H_block_err = zeros(N, N);

for i = 1:N
    rows = (i - 1) * nx + (1:nx);

    for j = 1:N
        cols = (j - 1) * nx + (1:nx);
        H_block_err(i,j) = norm(E_H(rows, cols), 'fro');
    end
end

figure(9); set(gcf, "Name", "H block error");
imagesc(H_block_err);
colorbar;
xlabel("Block j");
ylabel("Block i");
title("H block error");

%% 5. Plot predicted results and optimal solution
figure(10); clf; set(gcf, 'Name', 'Predicted states');
tiledlayout(7,1);

% y
nexttile();
plot(ltv.x_pred(:,1), 'rx-'); hold on;
plot(ltv.x0(1), 'r.', 'MarkerSize', 15)
plot(fuz.x_pred(:,1), 'bx-');
plot(fuz.x0(1), 'b.', 'MarkerSize', 15)
title("y")

% vy
nexttile();
plot(ltv.x_pred(:,2), 'rx-'); hold on;
plot(ltv.x0(2), 'r.', 'MarkerSize', 15)
plot(fuz.x_pred(:,2), 'bx-');
plot(fuz.x0(2), 'b.', 'MarkerSize', 15)
title("vy")

% psi
nexttile();
plot(ltv.x_pred(:,3), 'rx-'); hold on;
plot(ltv.x0(3), 'r.', 'MarkerSize', 15)
plot(fuz.x_pred(:,3), 'bx-');
plot(fuz.x0(3), 'b.', 'MarkerSize', 15)
title("psi")

% r
nexttile();
plot(ltv.x_pred(:,4), 'rx-'); hold on;
plot(ltv.x0(4), 'r.', 'MarkerSize', 15)
plot(fuz.x_pred(:,4), 'bx-');
plot(fuz.x0(4), 'b.', 'MarkerSize', 15)
title("r")

% delta
nexttile();
plot(ltv.x_pred(:,5), 'rx-'); hold on;
plot(ltv.x0(5), 'r.', 'MarkerSize', 15)
plot(fuz.x_pred(:,5), 'bx-');
plot(fuz.x0(5), 'b.', 'MarkerSize', 15)
title("delta")

% delta dot
nexttile();
plot(ltv.x_pred(:,6), 'rx-'); hold on;
plot(ltv.x0(6), 'r.', 'MarkerSize', 15)
plot(fuz.x_pred(:,6), 'bx-');
plot(fuz.x0(6), 'b.', 'MarkerSize', 15)
title("delta dot")

legend("ltv pred", "ltv actual", "fuz pred", "fuz actual")

% optimal solution delta
nexttile();
plot(ltv.u_opt(:), 'rx-'); hold on;
plot(fuz.u_opt(:), 'bx-');
title("opt delta")



%%



%% Aux funcitons
function mat = csv2mat(path, idx, rows, cols)

    data = readmatrix(path);
    data_row = data(idx,:);

    mat = reshape(data_row, rows, cols);    
end 