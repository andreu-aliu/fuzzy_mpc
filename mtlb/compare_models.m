cd('/home/andreu/ros_ws/src/as/control/fuzzy_mpc/mtlb'); addpath(genpath('/home/andreu/ros_ws/src/as/control/fuzzy_mpc/mtlb'))
clear all;
%% Setup
dataFile = "/home/andreu/SIMULATIONS/results_3/run_2/rosbag/rosbag_0.mcap";

% Simulation window
idx_start = 600;
horizon  = 200;

%% LOAD DATA
data = read_ros2bag(dataFile, 0.02);

%% TAKE A WINDOW OF DATA
idx_end = min(idx_start + horizon - 1, height(data.vx));

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
grid on

% TV moment
ax5 = nexttile(rightLayout); hold on;
plot(data.time, data.mz, 'w');
plot(meas.t, in.mz, 'b','LineWidth',1.2);
ylabel('Mz [Nm]');
title('TV Mz');
xlabel('Time [s]');

% Link time axes
linkaxes([ax1 ax2 ax3 ax4 ax5],'x');
grid on

%% COMPUTE MODELS ON WINDOW

% List of models to compare
models = {
    @anfis_delta, 'ANFIS delta';
    @ltv, 'LTV MPC';
    @ltv_tv, 'LTV MPC with TV';
};

% Run each model on the selected window
sim_results = cell(size(models,1),1);

for k = 1:size(models,1)
    modelFcn = models{k,1};
    name     = models{k,2};
    
    fprintf('Simulating model: %s\n', name);
    sim_results{k} = simulateModel(modelFcn, in, init_state);
end

% COMPUTE MEASURED VALUES IN THE LOCAL FRAME

% Time
t = meas.t;
t0 = t(1);
t_rel = t - t0;

% Use the selected window global trajectory
xg = meas.x(:);
yg = meas.y(:);

% Estimate heading from trajectory (robust for simulation logs)
dx = gradient(xg);
dy = gradient(yg);

psi_traj = unwrap(atan2(dy, dx));

% If the car is almost static / dx,dy ~ 0, heading may get noisy.
% In that case, you can fallback to integrating yaw-rate:
% dt_vec = [diff(t); mean(diff(t))];
% psi_traj = unwrap(cumsum(meas.r(:) .* dt_vec));

psi0 = psi_traj(1);

% Local position: translate to window origin, rotate by -psi0
x0 = xg(1);
y0 = yg(1);

dX = xg - x0;
dY = yg - y0;

x_local =  dX*cos(psi0) + dY*sin(psi0);
y_local = -dX*sin(psi0) + dY*cos(psi0);

% Local heading relative to initial heading
psi_local = unwrap(psi_traj - psi0);

% Package
meas_local.t   = t_rel;
meas_local.x   = x_local;
meas_local.y   = y_local;
meas_local.psi = psi_local;
meas_local.vy  = meas.vy(:);
meas_local.r   = meas.r(:);

% PLOT RESULTS (MEASURED vs MODELS)

figure('Name','Model vs Data (Local Frame)','Position',[100 100 1200 800]);
tl = tiledlayout(4,1,'TileSpacing','compact','Padding','compact');

% Prepare axes
axY   = nexttile(tl,1); hold on; grid on; ylabel('y_{local} [m]');  title('Lateral position');
axVy  = nexttile(tl,2); hold on; grid on; ylabel('v_y [m/s]'); title('Lateral velocity');
axPsi = nexttile(tl,3); hold on; grid on; ylabel('\psi_{local} [rad]'); title('Heading');
axR   = nexttile(tl,4); hold on; grid on; ylabel('r [rad/s]'); title('Yaw rate'); xlabel('t [s]');

% Plot measured first
plot(axY,   meas_local.t, meas_local.y,   'w', 'LineWidth', 2, 'DisplayName','Measured');
plot(axPsi, meas_local.t, meas_local.psi, 'w', 'LineWidth', 2, 'DisplayName','Measured');
plot(axVy,  meas_local.t, meas_local.vy,  'w', 'LineWidth', 2, 'DisplayName','Measured');
plot(axR,   meas_local.t, meas_local.r,   'w', 'LineWidth', 2, 'DisplayName','Measured');

% Overlay each model
for k = 1:size(models,1)
    name = models{k,2};
    states = sim_results{k};

    % Convert struct array -> vectors
    y_sim   = arrayfun(@(s) s.y,   states(:));
    psi_sim = arrayfun(@(s) s.psi, states(:));
    vy_sim  = arrayfun(@(s) s.vy,  states(:));
    r_sim   = arrayfun(@(s) s.r,   states(:));

    % Same time base
    t_sim = meas_local.t;  % (simulateModel uses same N and dt)

    plot(axY,   t_sim, y_sim,   'LineWidth', 1.4, 'DisplayName', name);
    plot(axPsi, t_sim, psi_sim, 'LineWidth', 1.4, 'DisplayName', name);
    plot(axVy,  t_sim, vy_sim,  'LineWidth', 1.4, 'DisplayName', name);
    plot(axR,   t_sim, r_sim,   'LineWidth', 1.4, 'DisplayName', name);
end

% Limits
% ylim(axY,[-1.5 1.5]);
% ylim(axPsi,[-0.8 0.8]);
% ylim(axVy,[-1 1]);
% ylim(axR,[-1 1]);

% Legends
legend(axY,   'Location','best');
legend(axPsi, 'Location','best');
legend(axVy,  'Location','best');
legend(axR,   'Location','best');

linkaxes([axY axPsi axVy axR],'x');

%%



%% Simulate model function
function states = simulateModel(modelFcn, inputs, init_state)
    dt = mean(diff(inputs.t));
    N  = length(inputs.t);
    
    % Convert init_state struct vector (+delta +delta_dot)
    x = [init_state.y, init_state.vy, init_state.psi, init_state.r, 0, 0]';
    
    states(N,1) = init_state;
    
    for k = 1:N
        % Get input
        u = [inputs.vx(k), inputs.st(k), inputs.mz(k)];
    
        % Log state
        states(k).y         = x(1);
        states(k).vy        = x(2);
        states(k).psi       = x(3);
        states(k).r         = x(4);
        states(k).delta     = x(5);
        states(k).delta_dot = x(6);
    
        x = modelFcn(x, u, dt);           % propagate
    end
end