cd /home/andreu/bcnemotorsport/adaptive_mpc/fuzzy-mpc/mtlb; addpath(genpath('/home/andreu/bcnemotorsport/adaptive_mpc/fuzzy-mpc/mtlb'))
clear all
%% Setup

data_path = "/home/andreu/bcnemotorsport/data/simu/rosbag2_2026_02_23-20_19_45";


%% Load data 
data = read_ros2bag(data_path)

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
plot(t, data.steering, 'LineWidth', 1)
ylabel('\delta [rad]')
title('Steering')
grid on

ax(2) = nexttile(tl);
plot(t, data.vx, 'LineWidth', 1)
ylabel('v_x [m/s]')
title('Longitudinal Velocity')
grid on

ax(3) = nexttile(tl);
plot(t, data.vy, 'LineWidth', 1)
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