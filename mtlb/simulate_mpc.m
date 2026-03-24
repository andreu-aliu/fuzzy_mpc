cd('/home/andreu/ros_ws/src/as/control/fuzzy_mpc/mtlb'); addpath(genpath('/home/andreu/ros_ws/src/as/control/fuzzy_mpc/mtlb'))
clear all;
%% Setup
dataFile = "/home/andreu/bcnemotorsport/data/simu/trackdrive_FSG";

% Simulation window
idx_start = 3000;
n  = 100;

%% LOAD DATA
data = read_ros2bag(dataFile);

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

init_state.y = 0;
init_state.vy = meas.vy(1);
init_state.psi = 0;
init_state.r = meas.r(1);

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
ylabel('\delta [rad]');
title('Steering');
xlabel('Time [s]');

% Link time axes
linkaxes([ax1 ax2 ax3 ax4],'x');
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

% Weights
params.q_y  = 1000000;
params.q_vy = 1;
params.q_psi= 1;
params.q_r  = 1;

params.p_y  = 50000;
params.p_vy = 5;
params.p_psi= 5;
params.p_r  = 5;

params.r_st = 0.2;
params.r_mz = 1000; 

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

%% SIMULATE

% Global state history: [x y psi vx vy r]
X = cell(n,1);
U = cell(n,1);

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

    % Closest trajectory point
    idx = find_closest_point(traj, Xg(1), Xg(2));

    % Convert GLOBAL → LOCAL MPC state
    x_0 = project_to_local_cartesian(traj, idx, meas, k);

    % Build reference
    [x_ref, vx_ref] = build_reference_local(traj, idx, dt, Np);

    % MPC solve
    x_ref_vec  = reshape(x_ref.', [], 1);
    x_pred_vec = reshape(x_pred.', [], 1);
    u_pred_vec = reshape(u_pred.', [], 1);

    [x_pred_vec, u_pred_vec] = mpc(x_0, x_ref_vec, x_pred_vec, u_pred_vec, vx_ref, params);

    % Debug plots 
    % mpc_debug_plot(k, x_0, x_ref, x_pred, u_pred, params, debug_opts);

    x_pred = reshape(x_pred_vec, nx, []).';
    u_pred = reshape(u_pred_vec, nu, []).';

    % Apply first control
    u = u_pred(1,:)';
    U{k+1} = u;

    % Simulate GLOBAL dynamics
    X{k+1} = sim_bicycleDynamic_linear(Xg', u', traj.vx(idx), dt)';

end
%% PLOT RESULTS

% Convert cell → matrix
Xg_mat = cell2mat(X')';   % N x 6
U_mat  = cell2mat(U')';   % N x 2

% Extract global states
x_sim   = Xg_mat(:,1);
y_sim   = Xg_mat(:,2);

st_sim = U_mat(:,1);
mz_sim = U_mat(:,2);

t_u = meas.t(1:size(U_mat,1));

figure('Name','MPC Results','Position',[100 100 1200 800]);
tl = tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

% ===== TOP: GLOBAL TRAJECTORY =====
ax1 = nexttile; hold on; grid on; axis equal;

% Reference trajectory (white solid)
plot(traj.x, traj.y, 'w', 'LineWidth', 2);

% Simulated path (white dashed)
plot(x_sim, y_sim, 'r', 'LineWidth', 2);

xlabel('X [m]');
ylabel('Y [m]');
title('Global Trajectory Tracking');

legend('Reference','Simulated','Location','best');

set(gca, 'Color', 'k');   % black background for contrast

% ===== BOTTOM: CONTROLS =====
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

function x_local = project_to_local_cartesian(traj, idx, meas, k)

    % Trajectory frame
    xt = traj.x(idx);
    yt = traj.y(idx);
    psi_t = traj.psi(idx);

    % Vehicle
    xv = meas.x(k);
    yv = meas.y(k);
    psi_v = meas.psi(k);

    % Relative position
    dx = xv - xt;
    dy = yv - yt;

    % Rotate into trajectory frame
    x_local_fwd =  cos(psi_t)*dx + sin(psi_t)*dy;
    y_local     = -sin(psi_t)*dx + cos(psi_t)*dy;

    % Heading error
    psi_err = wrapToPi(psi_v - psi_t);

    x_local = [y_local;
               meas.vy(k);
               psi_err;
               meas.r(k)];
end

function [x_ref, vx_ref] = build_reference_local(traj, idx0, dt, Np)

    s0   = traj.s(idx0);
    psi0 = traj.psi(idx0);
    x0   = traj.x(idx0);
    y0   = traj.y(idx0);

    x_ref  = zeros(Np,4);
    vx_ref = zeros(Np,1);

    s_i = s0;

    for i = 1:Np

        % Use trajectory speed
        vx_i = interp1(traj.s, traj.vx, s_i, 'linear');

        % Propagate arc-length using trajectory speed
        s_i = s_i + vx_i * dt;

        if s_i > traj.s(end)
            s_i = traj.s(end);
        end

        % Interpolate trajectory
        x_t   = interp1(traj.s, traj.x, s_i, 'spline');
        y_t   = interp1(traj.s, traj.y, s_i, 'spline');
        psi_t = interp1(traj.s, traj.psi, s_i, 'spline');

        % Transform into local frame
        dx = x_t - x0;
        dy = y_t - y0;

        y_local = -sin(psi0)*dx + cos(psi0)*dy;
        psi_rel = wrapToPi(psi_t - psi0);

        % Fill reference
        x_ref(i,1) = y_local;
        x_ref(i,2) = 0;
        x_ref(i,3) = psi_rel;
        x_ref(i,4) = 0;

        vx_ref(i) = vx_i;
    end
end