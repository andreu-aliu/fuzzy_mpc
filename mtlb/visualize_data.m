cd /home/andreu/bcnemotorsport/adaptive_mpc/fuzzy-mpc/mtlb; addpath(genpath('/home/andreu/bcnemotorsport/adaptive_mpc/fuzzy-mpc/mtlb'))
clear all
%% Setup

data_path = "/home/andreu/bcnemotorsport/data/simu/trackdrive_FSG";

%% Load data 
data = read_ros2bag(data_path);

%% Plot data
t = data.time;

figure('Color','k','Position',[100 100 1400 800])

tl = tiledlayout(5,2,'TileSpacing','compact','Padding','compact');

% ======================
% Left column — Map
% ======================
nexttile(tl,[5 1])
plot(data.x, data.y, 'w', 'LineWidth', 1.2)
axis equal
grid on
xlabel('x [m]')
ylabel('y [m]')
title('Vehicle Map')

% ======================
% Right column — Signals
% ======================
ax = gobjects(5,1);

ax(1) = nexttile(tl);
plot(t, data.st, 'LineWidth', 1)
ylabel('\delta [rad]')
title('Steering')
grid on

ax(2) = nexttile(tl);
plot(t, data.vx, 'LineWidth', 1)
ylabel('v_x [m/s]')
title('Longitudinal Velocity')
grid on

ax(3) = nexttile(tl);
plot(t, data.vy,'LineWidth', 1)
ylabel('v_y [m/s]')
title('Lateral Velocity')
grid on

ax(4) = nexttile(tl);
plot(t, data.mz, 'LineWidth', 1)
ylabel('M_z [Nm]')
title('Yaw Moment (Torque Vectoring)')
grid on

ax(5) = nexttile(tl);
plot(t, data.Tfl, 'LineWidth', 1); hold on
plot(t, data.Tfr, 'LineWidth', 1);
plot(t, data.Trl, 'LineWidth', 1);
plot(t, data.Trr, 'LineWidth', 1);
ylabel('Torque [Nm]')
xlabel('Time [s]')
title('Wheel Torques')
legend('FL','FR','RL','RR')
grid on

linkaxes(ax,'x')


%% Plot Delta_r Delta_vy and filtered
delta_r  = data.r(1:end-1) - data.r(2:end);
delta_vy = data.vy(1:end-1) - data.vy(2:end);

% Filter with Savitzky-Golay
r_filtered = sgolayfilt_custom(data.r, 3, 21);
vy_filtered = sgolayfilt_custom(data.vy, 3, 21);
delta_r_filtered = r_filtered(1:end-1) - r_filtered(2:end);
delta_vy_filtered = vy_filtered(1:end-1) - vy_filtered(2:end);

% Plot
figure()
tl = tiledlayout(4,1,'TileSpacing','compact','Padding','compact');
ax = gobjects(4,1);

% Yaw rate
ax(1) = nexttile(tl);
plot(t, data.r, 'LineWidth', 1)
hold on
plot(t, r_filtered, 'LineWidth', 1)
ylabel('\delta [rad]')
title('Yaw rate')
grid on

ax(2) = nexttile(tl);
plot(t(1:end-1), delta_r, 'LineWidth', 1)
hold on; 
plot(t(1:end-1), delta_r_filtered, 'LineWidth', 1)
ylabel('v_x [m/s]')
title('\Delta Yaw rate')
grid on

% Lateral velocity
ax(3) = nexttile(tl);
plot(t, data.vy, 'LineWidth', 1)
hold on
plot(t, vy_filtered, 'LineWidth', 1)
ylabel('\delta [rad]')
title('Yaw rate')
grid on

ax(4) = nexttile(tl);
plot(t(1:end-1), delta_vy, 'LineWidth', 1)
hold on; 
plot(t(1:end-1), delta_vy_filtered, 'LineWidth', 1)
ylabel('v_x [m/s]')
title('\Delta Yaw rate')
grid on

linkaxes(ax,'x')

%% Time stamps diff
figure
timeDiff = diff(t);
plot(timeDiff)