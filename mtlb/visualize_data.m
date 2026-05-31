cd('/home/andreu/ros_ws/src/as/control/fuzzy_mpc/mtlb'); addpath(genpath('/home/andreu/ros_ws/src/as/control/fuzzy_mpc/mtlb'))
clear all
%% Setup

data_path = "/home/andreu/SIMULATIONS/results_3/run_2/rosbag";

%% Load data 
data = read_ros2bag(data_path, 0.02);

%% Plot data
t = data.time;

figure('Color','k','Position',[100 100 1400 800])

tl = tiledlayout(6,2,'TileSpacing','compact','Padding','compact');

% ======================
% Left column — Map
% ======================
nexttile(tl,[6 1])
plot(data.x, data.y, 'w', 'LineWidth', 1.2)
axis equal
grid on
xlabel('x [m]')
ylabel('y [m]')
title('Vehicle Map')

% ======================
% Right column — Signals
% ======================
ax = gobjects(6,1);

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

ax(6) = nexttile(tl);
plot(t, data.r, 'LineWidth', 1)
ylabel('r [rad/s]')
xlabel('Time [s]')
title('Yaw rate')
grid on

linkaxes(ax,'x')

%% Check yaw rate (r) vs heading derivative
Ts = mean(diff(t));

if isfield(data,'heading')
    psi = unwrap(data.heading(:));
else
    dx = gradient(data.x(:));
    dy = gradient(data.y(:));
    psi = unwrap(atan2(dy, dx));
end

psi_dot = gradient(psi, Ts);

figure('Color','k','Position',[140 140 1400 700])
tl = tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

ax = nexttile(tl); hold on; grid on;
plot(t, data.r, 'LineWidth', 1.2);
plot(t, psi_dot, 'LineWidth', 1.2);
ylabel('[rad/s]')
title('Yaw rate check')
legend('r', 'd(heading)/dt', 'Location','best')

ax2 = nexttile(tl); hold on; grid on;
scatter(psi_dot, data.r(:), 6, 'filled', 'MarkerFaceAlpha', 0.15);
xlabel('d(heading)/dt [rad/s]')
ylabel('r [rad/s]')
title('r vs d(heading)/dt')

%% Check accelerations (ax, ay)
if isfield(data,'ax') && isfield(data,'ay')
    Ts = mean(diff(t));

    vx = data.vx(:);
    vy = data.vy(:);
    r  = data.r(:);
    ax_meas = data.ax(:);
    ay_meas = data.ay(:);

    vx_dot_fd = gradient(vx, Ts);
    vy_dot_fd = gradient(vy, Ts);

    % Body-frame kinematics:
    % ax ~= vx_dot - vy*r  -> vx_dot ~= ax + vy*r
    % ay ~= vy_dot + vx*r  -> vy_dot ~= ay - vx*r
    vx_dot_from_ax = ax_meas + vy.*r;
    vy_dot_from_ay = ay_meas - vx.*r;

    figure('Color','k','Position',[120 120 1400 800])
    tl = tiledlayout(2,1,'TileSpacing','compact','Padding','compact');
    ax = gobjects(2,1);

    ax(1) = nexttile(tl); hold on; grid on;
    plot(t, ax_meas, 'LineWidth', 1.1);
    plot(t, vx_dot_fd, 'LineWidth', 1.1);
    plot(t, vx_dot_from_ax, 'LineWidth', 1.1);
    ylabel('[m/s^2]');
    title('v_x derivative check');
    legend('a_x', 'd(v_x)/dt', 'a_x + v_y r', 'Location','best');

    ax(2) = nexttile(tl); hold on; grid on;
    plot(t, ay_meas, 'LineWidth', 1.1);
    plot(t, vy_dot_fd, 'LineWidth', 1.1);
    plot(t, vy_dot_from_ay, 'LineWidth', 1.1);
    ylabel('[m/s^2]');
    title('v_y derivative check');
    xlabel('Time [s]');
    legend('a_y', 'd(v_y)/dt', 'a_y - v_x r', 'Location','best');

    linkaxes(ax,'x')
else
    warning('No data.ax/data.ay fields found. Update read_ros2bag output and re-load.');
end


%% Plot Delta_r Delta_vy and filtered
delta_r  = data.r(1:end-1) - data.r(2:end);
delta_vy = data.vy(1:end-1) - data.vy(2:end);

% Filter with Savitzky-Golay
r_filtered = sgolayfilt_custom(data.r, 3, 11);
vy_filtered = sgolayfilt_custom(data.vy, 3, 11);
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
title('Vy')
grid on

ax(4) = nexttile(tl);
plot(t(1:end-1), delta_vy, 'LineWidth', 1)
hold on; 
plot(t(1:end-1), delta_vy_filtered, 'LineWidth', 1)
ylabel('v_x [m/s]')
title('\Delta Vy')
grid on

linkaxes(ax,'x')

%% Time stamps diff
figure
timeDiff = diff(t);
plot(timeDiff)
