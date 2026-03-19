cd('/home/andreu/ros_ws/src/as/control/fuzzy_mpc/mtlb'); addpath(genpath('/home/andreu/ros_ws/src/as/control/fuzzy_mpc/mtlb'))
clear all;
%% Setup
dataFile = "/home/andreu/bcnemotorsport/data/simu/trackdrive_FSG";

% Simulation window
idx_start = 450;
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
%% MPC PARAMETERS
% Weights
params.q_y  = 1000000;
params.q_vy = 1;
params.q_psi= 1;
params.q_r  = 1;

params.p_y  = 50000
params.p_vy = 5;
params.p_psi= 5;
params.p_r  = 5;

params.r_st = 0.2;
params.r_mz = 1000; 

% Bounds
params.min_st = -0.436;
params.max_st = 0.436;
params.min_mz = -1000;
params.max_mz = 1000;

%% SIMULATE

% Global state matrix
X = cell(n);
U = cell(n);

% First iteration initialization
x_0 = [0;meas.vy(idx_start);0;meas.r(idx_start)];
x_pred = [zeros(n,1);meas.vy;zeros(n,1);meas.r];
u_pred = zeros(n,2);

% Simulation loop
for k = 1:n-1

    % Prepare reference to the local frame
    
    x_ref = 

    % MPC 
    [x_pred, u_pred] = mpc(x_0, x_ref, x_pred, u_pred, meas.vx(i), params);
    U{k+1} = [u_pred(1); u_pred(2)];

    % Simulation
    X{k+1} = [];

end


%% PLOT