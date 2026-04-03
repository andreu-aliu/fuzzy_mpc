cd('/home/andreu/ros_ws/src/as/control/fuzzy_mpc/mtlb'); addpath(genpath('/home/andreu/ros_ws/src/as/control/fuzzy_mpc/mtlb'))
clear all;
%% Setup
dataFile = "/home/andreu/bcnemotorsport/data/simu/trackdrive_FSG";

% Simulation window (from data)
idx_start = 3000;
n  = 300;

%% LOAD DATA
data = read_ros2bag(dataFile, 0.02);

%% TAKE A WINDOW OF DATA
idx_end = min(idx_start + n - 1, height(data.vx));

meas.t = data.time(idx_start:idx_end);
in.t = data.time(idx_start:idx_end);

in.st = data.st(idx_start:idx_end);
in.mz = data.mz(idx_start:idx_end);
in.vx = data.vx(idx_start:idx_end);

meas.x = data.x(idx_start:idx_end);
meas.y = data.y(idx_start:idx_end);
meas.vx = data.vx(idx_start:idx_end);
meas.vy = data.vy(idx_start:idx_end);
meas.r = data.r(idx_start:idx_end);

meas.psi = compute_path_heading(meas.x, meas.y);

%% CHECK DATA LOADED
figure('Name','Data check','Position',[100 100 1200 600]);

% Main layout: 1 row, 2 columns
mainLayout = tiledlayout(1,2);

% --- Left: trajectory map ---
nexttile(mainLayout,1); hold on; grid on; axis equal;
plot(data.x, data.y, 'w');                    % full run
plot(meas.x, meas.y, 'b', 'LineWidth',1.5);                   % selected window
xlabel('X [m]'); ylabel('Y [m]');
title('Trajectory map');
legend('Full run','Selected window');

% --- Right: nested stacked time series ---
rightLayout = tiledlayout(mainLayout,5,1,'TileSpacing','compact','Padding','compact');
rightLayout.Layout.Tile = 2; 

% Longitudinal velocity
ax1 = nexttile(rightLayout); hold on;
plot(data.time, data.vx, 'w');
plot(meas.t, in.vx, 'b','LineWidth',1.2);
ylabel('V_x [m/s]');
title('Longitudinal velocity');
grid on

% Lateral velocity
ax2 = nexttile(rightLayout); hold on;
plot(data.time, data.vy, 'w');
plot(meas.t, meas.vy, 'b','LineWidth',1.2);
ylabel('V_y [m/s]');
title('Lateral velocity');
grid on

% Yaw rate
ax3 = nexttile(rightLayout); hold on;
plot(data.time, data.r, 'w');
plot(meas.t, meas.r, 'b','LineWidth',1.2);
ylabel('r [rad/s]');
title('Yaw rate');
grid on

% Steering
ax4 = nexttile(rightLayout); hold on;
plot(data.time, data.st, 'w');
plot(meas.t, in.st, 'b','LineWidth',1.2);
ylabel('\delta [rad]');
title('Steering');
xlabel('Time [s]');

% TV moment
ax5 = nexttile(rightLayout); hold on;
plot(data.time, data.mz, 'w');
plot(meas.t, in.mz, 'b','LineWidth',1.2);
ylabel('M_z [Nm]');
title('Yaw moment');
xlabel('Time [s]');

% Link time axes
linkaxes([ax1 ax2 ax3 ax4 ax5],'x');
grid on

%% BUILD TRAJECTORY 

ds = 0.025;  % [m]

% Compute cumulative arc-length
dx = diff(meas.x);
dy = diff(meas.y);
s  = [0; cumsum(sqrt(dx.^2 + dy.^2))];

% Uniform arc-length grid
s_uniform = (0:ds:s(end)).';

% Interpolate
x_u = interp1(s, meas.x, s_uniform, 'spline');
y_u = interp1(s, meas.y, s_uniform, 'spline');
vx_u = interp1(s, meas.vx, s_uniform, 'spline');

% Compute heading
dx_u = gradient(x_u, ds);
dy_u = gradient(y_u, ds);
psi_u = unwrap(atan2(dy_u, dx_u));

traj.s   = s_uniform;
traj.x   = x_u;
traj.y   = y_u;
traj.psi = psi_u;
traj.vx  = vx_u;


%% MPC PARAMETERS & OPTIONS

Np = 60;
nx = 4;
nu = 2;
dt = 0.01;

% Scales for normalization
params.scale_y   = 0.3;   % m
params.scale_vy  = 0.1;   % m/s
params.scale_psi = 0.05;  % rad
params.scale_r   = 0.5;   % rad/s
params.scale_st  = 0.2;   % rad
params.scale_mz  = 1000;  % Nm

% Weights
params.q_y  = 800;
params.q_vy = 0;
params.q_psi= 0;
params.q_r  = 0;

params.p_y  = 8000;
params.p_vy = 0;
params.p_psi= 0;
params.p_r  = 0;

params.r_st = 0;
params.r_mz = 0; 

% Bounds
params.min_st = -0.436;
params.max_st = 0.436;
params.min_mz = -0;
params.max_mz = 0;

% Debug options
debug_opts.enabled = true;
debug_opts.step    = [];     % [] = all, or e.g. 20
debug_opts.pause   = false;  % true = step-by-step
debug_opts.figure_id = 99;

% Compare options
comp_opts.enabled = false;
comp_opts.step    = [];     % [] = all, or e.g. 20
comp_opts.pause   = false;  % true = step-by-step
comp_opts.figure_id = 98;

%% SIMULATE 

% Global state history: [x y psi vx vy r]
X = cell(n,1);
U = cell(n,1);
model_error = NaN(1,n);

% Initial GLOBAL state
X{1} = [meas.x(1);
        meas.y(1);
        meas.psi(1);
        meas.vx(1);
        meas.vy(1);
        meas.r(1)];

U{1} = [in.st(1); in.mz(1)];

% Warm start
x_pred = repmat([0 meas.vy(1) 0 meas.r(1)], Np, 1); % local state
u_pred = zeros(Np, 2);

for k = 1:n-1

    % Current global state
    Xg = X{k};

    % Find global reference
    X_ref_global = build_reference_global(traj, Xg, dt, Np+1);

    % Convert GLOBAL → LOCAL MPC state
    [x_0, ~] = global_to_local_state(Xg, X_ref_global{1});
    for i = 1:Np
        [x_ref(i,:), vx_ref(i)] = global_to_local_state(X_ref_global{i+1}, X_ref_global{1});
    end

    % MPC solve
    x_ref_vec  = reshape(x_ref.', [], 1);
    x_pred_vec = reshape(x_pred.', [], 1);
    u_pred_vec = reshape(u_pred.', [], 1);

    [x_pred_vec, u_pred_vec, ~] = mpc(x_0, x_ref_vec, x_pred_vec, u_pred_vec, vx_ref, params);

    x_pred = reshape(x_pred_vec, nx, []).';
    u_pred = reshape(u_pred_vec, nu, []).';

    % Debug plots 
    mpc_debug_plot(k, x_0, x_ref, x_pred, u_pred, params, debug_opts);

    % Apply first control
    u = u_pred(1,:)';
    % u = [in.st(k);in.mz(k)]; % Test with measured inputs

    U{k+1} = u;

    % Simulate GLOBAL dynamics
    % X{k+1} = sim_anfis_direct(Xg', u', vx_ref(1), dt)';
    X{k+1} = sim_bicycleDynamic_linear(Xg', u', vx_ref(1), dt)';
    % X{k+1} = sim_ltv(Xg', u', vx_ref(1), dt);

    % Compare last seen states with mpc predicted. Model error
    if k > Np
        X_compare = X(k-Np+1:k+1);
        U_compare = U(k-Np+1:k);
        
        % Convert to local
        x_comp = NaN(Np,4);
        u_comp = NaN(Np,2);
        vx_comp = NaN(Np,1);
        x_0_comp = global_to_local_state(X_compare{1}, X_compare{1});
        for i = 1:Np
            [x_comp(i,:), vx_comp(i)] = global_to_local_state(X_compare{i+1}, X_compare{1});
            u_comp(i,:) = U_compare{i}';
        end

        % Use MPC function again to compute the predicted with 
        x_comp_vec  = reshape(x_comp.', [], 1);
        u_comp_vec  = reshape(u_comp.', [], 1);

        [~,~,x_pred_comp_vec] = mpc(x_0_comp, x_comp_vec, x_comp_vec, u_comp_vec, vx_comp, params);

        x_pred_comp = reshape(x_pred_comp_vec, nx, []).';

        % Norm difference
        err = reshape(x_pred_comp_vec - x_comp_vec, 4, []);

        err(1,:) = err(1,:) / params.scale_y;
        err(2,:) = err(2,:) / params.scale_vy;
        err(3,:) = err(3,:) / params.scale_psi;
        err(4,:) = err(4,:) / params.scale_r;
        
        model_error(k) = mean(vecnorm(err,2,1));
        fprintf("Model error: %.6f\n", model_error(k));

        % Comparation plots
        mpc_debug_plot(k, x_0_comp, x_comp, x_pred_comp, u_comp, params, comp_opts);
    end

    fprintf("Iteration %i done\n", k);
end
%% PLOT RESULTS

% Convert cell → matrix
Xg_mat = cell2mat(X')';   % N x 6
U_mat  = cell2mat(U')';   % N x 2

% Extract global states
x_sim = Xg_mat(:,1);
y_sim = Xg_mat(:,2);

st_sim = U_mat(:,1);
mz_sim = U_mat(:,2);

t_u = meas.t(1:size(U_mat,1));

figure('Name','MPC Results','Position',[100 100 1200 800]);
tl = tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

% ===== TOP: GLOBAL TRAJECTORY =====
ax1 = nexttile; hold on; grid on; axis equal;

% Reference path from measurements (white solid)
plot(meas.x, meas.y, 'w', 'LineWidth', 2);

% Simulated path (white dashed)
plot(x_sim, y_sim, '-r', 'LineWidth', 2);

xlabel('X [m]');
ylabel('Y [m]');
title('Global Trajectory Tracking');

legend('Reference (meas)','Simulated','Location','best');

set(gca, 'Color', 'k');   % black background

% ===== MIDDLE: CONTROLS =====
ax2 = nexttile; hold on; grid on;

yyaxis left
plot(t_u, st_sim, 'LineWidth', 1.5);
ylabel('\delta [rad]');

yyaxis right
plot(t_u, mz_sim, 'LineWidth', 1.5);
ylabel('M_z [Nm]');

xlabel('Time [s]');
title('Control Inputs');
legend('Steering','Yaw moment','Location','best');

% ===== BOTTOM: MODEL ERROR =====
ax3 = nexttile; hold on; grid on;

plot(t_u, model_error, 'LineWidth', 1.5);
ylabel('Vectorn norm difference');

xlabel('Time [s]');
title('Model Error');
legend('Model error','Location','best');

linkaxes([ax2 ax3],'x')


%%








%% AUXILIAR FUNCTIONS

function psi_path = compute_path_heading(x, y)
%COMPUTE_PATH_HEADING Estimate heading of the path from x-y trajectory

    dx = gradient(x);
    dy = gradient(y);

    psi_path = unwrap(atan2(dy, dx));
end

function idx = find_closest_point(traj, x, y)

    dx = traj.x - x;
    dy = traj.y - y;

    [~, idx] = min(dx.^2 + dy.^2);
end

function X_ref = build_reference_global(traj, Xg, dt, Np)
% Build reference state list in global coordinates
% Global state: [x y psi vx vy r]

    X_ref = cell(Np,1);
    
    % Find point closest to actual position
    idx = find_closest_point(traj, Xg(1), Xg(2));

    s_i = traj.s(idx);
    for i = 1:Np

        if s_i > traj.s(end)
            s_i = traj.s(end);
        end

        % Use trajectory speed
        vx_i = interp1(traj.s, traj.vx, s_i, 'linear');

        % Interpolate trajectory
        x_t   = interp1(traj.s, traj.x, s_i, 'spline');
        y_t   = interp1(traj.s, traj.y, s_i, 'spline');
        psi_t = interp1(traj.s, traj.psi, s_i, 'spline');

        % Propagate arc-length using trajectory speed
        s_i = s_i + vx_i * dt;

        % Build global reference states
        X_ref{i} = [x_t; y_t; psi_t; vx_i; 0; 0];
    end
end

function [x_local, vx] = global_to_local_state(X_global, X_ref)
% Convert global state into local frame defined by X_ref
% Global state: [x y psi vx vy r]
% Local state:  [y vy psi r]

    % Extract global state
    x = X_global(1);
    y = X_global(2);
    psi = X_global(3);
    vx = X_global(4);
    vy = X_global(5);
    r = X_global(6);

    % Extract reference state
    x_ref = X_ref(1);
    y_ref = X_ref(2);
    psi_ref = X_ref(3);

    % Relative position
    dx = x - x_ref;
    dy = y - y_ref;

    % Rotation: global → local (reference frame)
    x_local_pos =  cos(psi_ref)*dx + sin(psi_ref)*dy;
    y_local_pos = -sin(psi_ref)*dx + cos(psi_ref)*dy;

    % Relative heading
    psi_rel = wrapToPi(psi - psi_ref);

    % Assemble output
    x_local = zeros(4,1);
    x_local(1) = y_local_pos;
    x_local(2) = vy;
    x_local(3) = psi_rel;
    x_local(4) = r;

end

