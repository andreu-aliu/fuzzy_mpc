%% Inspect one rosbag and validate its model-identification data

clear;
clc;

mtlb_dir = '/home/andreu/ros_ws/src/as/control/fuzzy_mpc/mtlb';
cd(mtlb_dir);
addpath(genpath(mtlb_dir));

%% Configuration

data_path = "/media/andreu/200GB Toshiba/TFM Data/2026-07-05/05-07-2026__run_3";
Ts = 0.02;

filter_order = 3;
filter_windows = [5, 11, 21];
max_lag_seconds = 0.40;

% Leave empty to automatically show ten seconds around the strongest
% measured-steering change. Example manual selection: [35, 45].
raw_comparison_window = [];
raw_comparison_duration = 10;

%% Load processed signals and raw timing diagnostics

[data, audit] = read_ros2bag(data_path, Ts);
t = data.time(:);

fprintf('\nDATA AUDIT: %s\n', data_path);
fprintf('Processed duration: %.2f s | samples: %d | requested Ts: %.4f s\n', ...
    t(end) - t(1), numel(t), Ts);

%% Raw topic timing and interpolation quality

timing_table = make_timing_table(audit);
fprintf('\nRaw topic timing and interpolation quality:\n');
disp(timing_table);
fprintf(['Timing values above use rosbag MessageList.Time. Header-stamp ' ...
    'alignment must be checked separately if those messages provide headers.\n']);

if any(timing_table.Status ~= "OK")
    warning(['At least one topic needs timing inspection. Look for duplicate ' ...
        'timestamps, gaps above 3*Ts, or large distance from raw samples.']);
end

%% Processed signal integrity and excitation

signal_names = {'vx','vy','r','ax','ay','delta','st','mz', ...
    'Tfl','Tfr','Trl','Trr'};
signal_table = make_signal_table(data, signal_names);

fprintf('Processed signal integrity:\n');
disp(signal_table);

if any(~signal_table.AllFinite) || any(signal_table.SampleCount ~= numel(t))
    error('Processed signals contain non-finite data or inconsistent lengths.');
end

dt_error = max(abs(diff(t) - Ts));
fprintf('Maximum processed-grid timing error: %.3g s\n', dt_error);

excited = abs(data.delta) > 0.03 | abs(data.r) > 0.15 | abs(data.mz) > 20;
fprintf(['Lateral excitation: %.1f %% of samples satisfy |delta| > 0.03 rad, ' ...
    '|r| > 0.15 rad/s, or |Mz| > 20 Nm.\n'], 100*mean(excited));

%% Physical consistency and synchronization

vx = data.vx(:);
vy = data.vy(:);
r = data.r(:);
ax_measured = data.ax(:);
ay_measured = data.ay(:);

vx_dot = gradient(vx, Ts);
vy_dot = gradient(vy, Ts);
r_dot = gradient(r, Ts);

% Body-frame kinematics.
ax_reconstructed = vx_dot - vy.*r;
ay_reconstructed = vy_dot + vx.*r;

[ax_correlation, ax_nrmse, ax_bias] = consistency_metrics( ...
    ax_measured, ax_reconstructed);
[ay_correlation, ay_nrmse, ay_bias] = consistency_metrics( ...
    ay_measured, ay_reconstructed);

max_lag_samples = round(max_lag_seconds / Ts);
[st_delta_correlation, st_delta_lag] = best_lag( ...
    data.st, data.delta, max_lag_samples, false);
[delta_r_correlation, delta_r_lag] = best_lag( ...
    data.delta, data.r, max_lag_samples, true);
[mz_rdot_correlation, mz_rdot_lag] = best_lag( ...
    data.mz, r_dot, max_lag_samples, true);
[ay_correlation_lagged, ay_lag] = best_lag( ...
    ay_measured, ay_reconstructed, max_lag_samples, false);

physical_table = table( ...
    ["ax vs dvx/dt - vy*r"; "ay vs dvy/dt + vx*r"], ...
    [ax_correlation; ay_correlation], ...
    [ax_nrmse; ay_nrmse], ...
    [ax_bias; ay_bias], ...
    'VariableNames', {'Check','Correlation','NormalizedRMSE','Bias'});

lag_table = table( ...
    ["steering command -> measured steering"; ...
     "measured steering -> yaw rate"; ...
     "yaw moment -> yaw acceleration"; ...
     "measured ay -> reconstructed ay"], ...
    [st_delta_correlation; delta_r_correlation; ...
     mz_rdot_correlation; ay_correlation_lagged], ...
    1000*Ts*[st_delta_lag; delta_r_lag; mz_rdot_lag; ay_lag], ...
    'VariableNames', {'Relationship','Correlation','ResponseLag_ms'});

fprintf('Physical consistency (zero lag):\n');
disp(physical_table);
fprintf('Synchronization/response lag estimates:\n');
disp(lag_table);
fprintf(['Positive lag means the second signal follows the first. For physical ' ...
    'input/output pairs, this includes real actuator and vehicle response delay.\n']);

%% Overview

figure('Name','Data overview','Color','w','Position',[100 100 1450 850]);
overview_layout = tiledlayout(6,2,'TileSpacing','compact','Padding','compact');

nexttile(overview_layout,[6 1]);
plot(data.x, data.y, 'LineWidth', 1.2);
axis equal;
grid on;
xlabel('x [m]');
ylabel('y [m]');
title('Vehicle trajectory');

overview_axes = gobjects(6,1);
overview_axes(1) = nexttile(overview_layout);
plot(t, data.st, 'LineWidth', 1); hold on;
plot(t, data.delta, 'LineWidth', 1);
ylabel('[rad]'); title('Steering'); legend('Command','Measured'); grid on;

overview_axes(2) = nexttile(overview_layout);
plot(t, vx, 'LineWidth', 1);
ylabel('v_x [m/s]'); title('Longitudinal velocity'); grid on;

overview_axes(3) = nexttile(overview_layout);
plot(t, vy, 'LineWidth', 1);
ylabel('v_y [m/s]'); title('Lateral velocity'); grid on;

overview_axes(4) = nexttile(overview_layout);
plot(t, data.mz, 'LineWidth', 1);
ylabel('M_z [Nm]'); title('Yaw moment'); grid on;

overview_axes(5) = nexttile(overview_layout);
plot(t, data.Tfl, t, data.Tfr, t, data.Trl, t, data.Trr, ...
    'LineWidth', 1);
ylabel('Torque [Nm]'); title('Wheel torques');
legend('FL','FR','RL','RR'); grid on;

overview_axes(6) = nexttile(overview_layout);
plot(t, r, 'LineWidth', 1);
ylabel('r [rad/s]'); xlabel('Time [s]'); title('Yaw rate'); grid on;
linkaxes(overview_axes,'x');

%% Raw timestamp plots

topic_names = ["State","Measured steering","Steering command","Torque vectoring"];
raw_times = {audit.raw.state.time, audit.raw.steering.time, ...
    audit.raw.steering_command.time, audit.raw.torque_vectoring.time};

figure('Name','Raw timing audit','Color','w','Position',[120 120 1450 850]);
timing_layout = tiledlayout(3,2,'TileSpacing','compact','Padding','compact');
for topic_idx = 1:numel(topic_names)
    nexttile(timing_layout);
    raw_dt = diff(raw_times{topic_idx});
    plot(raw_times{topic_idx}(2:end), 1000*raw_dt, '.-'); hold on;
    yline(1000*Ts, '--', 'Requested Ts');
    yline(3000*Ts, ':r', '3 Ts');
    xlabel('Time [s]'); ylabel('Raw dt [ms]');
    title(topic_names(topic_idx)); grid on;
end

nearest_fields = {'state','steering','steering_command','torque_vectoring'};
nearest_axis = nexttile(timing_layout,[1 2]); hold on;
for topic_idx = 1:numel(nearest_fields)
    plot(t, 1000*audit.nearest_distance.(nearest_fields{topic_idx}), ...
        'DisplayName', topic_names(topic_idx));
end
yline(1000*Ts, ':r', 'Ts');
xlabel('Time [s]'); ylabel('Nearest raw sample [ms]');
title('Distance from each resampled point to the nearest raw message');
legend('Location','best'); grid on;

%% Raw samples over resampled signals

if isempty(raw_comparison_window)
    steering_rate = abs(gradient(data.delta, Ts));
    [~, strongest_idx] = max(steering_rate);
    window_center = t(strongest_idx);
    window_start = max(t(1), window_center - raw_comparison_duration/2);
    window_end = min(t(end), window_start + raw_comparison_duration);
    window_start = max(t(1), window_end - raw_comparison_duration);
else
    validateattributes(raw_comparison_window, {'numeric'}, ...
        {'vector','numel',2,'increasing'}, mfilename, 'raw_comparison_window');
    window_start = raw_comparison_window(1);
    window_end = raw_comparison_window(2);
end

figure('Name','Raw versus resampled','Color','w','Position',[140 140 1450 850]);
raw_layout = tiledlayout(4,1,'TileSpacing','compact','Padding','compact');
raw_axes = gobjects(4,1);

raw_axes(1) = nexttile(raw_layout);
plot(t, r, 'LineWidth', 1.2); hold on;
plot(audit.raw.state.time, audit.raw.state.r, '.', 'MarkerSize', 7);
ylabel('r [rad/s]'); title('Yaw rate'); legend('Resampled','Raw'); grid on;

raw_axes(2) = nexttile(raw_layout);
plot(t, data.delta, 'LineWidth', 1.2); hold on;
plot(audit.raw.steering.time, audit.raw.steering.value, '.', 'MarkerSize', 7);
ylabel('\delta [rad]'); title('Measured steering'); legend('Resampled','Raw'); grid on;

raw_axes(3) = nexttile(raw_layout);
plot(t, data.st, 'LineWidth', 1.2); hold on;
plot(audit.raw.steering_command.time, audit.raw.steering_command.value, ...
    '.', 'MarkerSize', 7);
ylabel('\delta_{cmd} [rad]'); title('Steering command');
legend('Linearly resampled','Raw'); grid on;

raw_axes(4) = nexttile(raw_layout);
plot(t, data.mz, 'LineWidth', 1.2); hold on;
plot(audit.raw.torque_vectoring.time, audit.raw.torque_vectoring.mz, ...
    '.', 'MarkerSize', 7);
ylabel('M_z [Nm]'); xlabel('Time [s]'); title('Yaw moment');
legend('Linearly resampled','Raw'); grid on;

linkaxes(raw_axes,'x');
xlim(raw_axes(1), [window_start window_end]);

%% Physical consistency plots

figure('Name','Physical consistency','Color','w','Position',[160 160 1450 850]);
physical_layout = tiledlayout(2,2,'TileSpacing','compact','Padding','compact');

nexttile(physical_layout); hold on;
plot(t, ax_measured, 'LineWidth', 1);
plot(t, ax_reconstructed, 'LineWidth', 1);
xlabel('Time [s]'); ylabel('[m/s^2]'); title('Longitudinal acceleration');
legend('Measured a_x','dv_x/dt - v_y r'); grid on;

nexttile(physical_layout); hold on;
plot(t, ay_measured, 'LineWidth', 1);
plot(t, ay_reconstructed, 'LineWidth', 1);
xlabel('Time [s]'); ylabel('[m/s^2]'); title('Lateral acceleration');
legend('Measured a_y','dv_y/dt + v_x r'); grid on;

nexttile(physical_layout);
scatter(ax_reconstructed, ax_measured, 7, 'filled', 'MarkerFaceAlpha',0.15);
hold on; plot_identity_line(ax_reconstructed, ax_measured);
xlabel('Reconstructed a_x [m/s^2]'); ylabel('Measured a_x [m/s^2]');
title(sprintf('a_x: corr %.3f, NRMSE %.2f', ax_correlation, ax_nrmse)); grid on;

nexttile(physical_layout);
scatter(ay_reconstructed, ay_measured, 7, 'filled', 'MarkerFaceAlpha',0.15);
hold on; plot_identity_line(ay_reconstructed, ay_measured);
xlabel('Reconstructed a_y [m/s^2]'); ylabel('Measured a_y [m/s^2]');
title(sprintf('a_y: corr %.3f, NRMSE %.2f', ay_correlation, ay_nrmse)); grid on;

%% Yaw rate versus trajectory course rate

dx = gradient(data.x(:), Ts);
dy = gradient(data.y(:), Ts);
course_angle = unwrap(atan2(dy, dx));
course_rate = gradient(course_angle, Ts);

figure('Name','Course-rate check','Color','w','Position',[180 180 1400 700]);
course_layout = tiledlayout(2,1,'TileSpacing','compact','Padding','compact');
nexttile(course_layout); hold on;
plot(t, r, 'LineWidth', 1.1);
plot(t, course_rate, 'LineWidth', 1.1);
ylabel('[rad/s]'); title('Yaw rate and trajectory course rate');
legend('Vehicle yaw rate r','d(course angle)/dt'); grid on;

nexttile(course_layout);
scatter(course_rate, r, 7, 'filled', 'MarkerFaceAlpha',0.15);
xlabel('d(course angle)/dt [rad/s]'); ylabel('r [rad/s]'); grid on;
title('These differ when sideslip angle changes; this is not a strict equality check');

%% ANFIS one-step targets and smoothing sensitivity

figure('Name','Filtering and ANFIS targets','Color','w','Position',[200 200 1450 850]);
filter_layout = tiledlayout(2,2,'TileSpacing','compact','Padding','compact');

nexttile(filter_layout); hold on;
plot(t, r, 'DisplayName','Raw');
nexttile(filter_layout); hold on;
plot(t(1:end-1), diff(r), 'DisplayName','Raw');
nexttile(filter_layout); hold on;
plot(t, vy, 'DisplayName','Raw');
nexttile(filter_layout); hold on;
plot(t(1:end-1), diff(vy), 'DisplayName','Raw');

for window = filter_windows
    if window <= filter_order || mod(window,2) == 0 || window > numel(t)
        warning('Skipping invalid filter window %d.', window);
        continue;
    end
    r_filtered = sgolayfilt_custom(r, filter_order, window);
    vy_filtered = sgolayfilt_custom(vy, filter_order, window);
    label = sprintf('SG window %d (%.2f s)', window, window*Ts);

    nexttile(filter_layout,1); plot(t, r_filtered, 'DisplayName',label);
    nexttile(filter_layout,2); plot(t(1:end-1), diff(r_filtered), ...
        'DisplayName',label);
    nexttile(filter_layout,3); plot(t, vy_filtered, 'DisplayName',label);
    nexttile(filter_layout,4); plot(t(1:end-1), diff(vy_filtered), ...
        'DisplayName',label);
end

filter_titles = {'Yaw rate','ANFIS target: r(k+1)-r(k)', ...
    'Lateral velocity','ANFIS target: v_y(k+1)-v_y(k)'};
filter_ylabels = {'r [rad/s]','\Delta r [rad/s]', ...
    'v_y [m/s]','\Delta v_y [m/s]'};
for tile_idx = 1:4
    axis_handle = nexttile(filter_layout,tile_idx);
    title(axis_handle,filter_titles{tile_idx});
    ylabel(axis_handle,filter_ylabels{tile_idx});
    xlabel(axis_handle,'Time [s]');
    grid(axis_handle,'on');
    legend(axis_handle,'Location','best');
end

%% Lag-correlation plots

[lag_seconds, st_delta_curve] = lag_curve( ...
    data.st, data.delta, max_lag_samples, Ts);
[~, delta_r_curve] = lag_curve(data.delta, data.r, max_lag_samples, Ts);
[~, mz_rdot_curve] = lag_curve(data.mz, r_dot, max_lag_samples, Ts);
[~, ay_curve] = lag_curve(ay_measured, ay_reconstructed, max_lag_samples, Ts);

figure('Name','Lag correlations','Color','w','Position',[220 220 1450 800]);
lag_layout = tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
plot_lag_tile(nexttile(lag_layout), lag_seconds, st_delta_curve, ...
    'Steering command -> measured steering');
plot_lag_tile(nexttile(lag_layout), lag_seconds, delta_r_curve, ...
    'Measured steering -> yaw rate');
plot_lag_tile(nexttile(lag_layout), lag_seconds, mz_rdot_curve, ...
    'Yaw moment -> yaw acceleration');
plot_lag_tile(nexttile(lag_layout), lag_seconds, ay_curve, ...
    'Measured a_y -> reconstructed a_y');

%%

%% Local functions

function timing_table = make_timing_table(audit)
topics = ["State";"Measured steering";"Steering command";"Torque vectoring"];
timing = {audit.timing.state; audit.timing.steering; ...
    audit.timing.steering_command; audit.timing.torque_vectoring};
nearest = {audit.nearest_distance.state; audit.nearest_distance.steering; ...
    audit.nearest_distance.steering_command; ...
    audit.nearest_distance.torque_vectoring};

n = numel(topics);
messages = zeros(n,1); rate = zeros(n,1); median_dt = zeros(n,1);
p95_dt = zeros(n,1); max_gap = zeros(n,1); duplicates = zeros(n,1);
backward = zeros(n,1); gaps = zeros(n,1); p95_nearest = zeros(n,1);
max_nearest = zeros(n,1); status = strings(n,1);

for i = 1:n
    stats = timing{i};
    messages(i) = stats.message_count;
    rate(i) = stats.median_rate_hz;
    median_dt(i) = 1000*stats.median_dt;
    p95_dt(i) = 1000*stats.p95_dt;
    max_gap(i) = 1000*stats.max_dt;
    duplicates(i) = stats.duplicate_count;
    backward(i) = stats.backward_count;
    gaps(i) = stats.gaps_over_3Ts;
    p95_nearest(i) = 1000*percentile_value(nearest{i},95);
    max_nearest(i) = 1000*max(nearest{i});

    needs_check = backward(i) > 0 || duplicates(i) > 0 || ...
        stats.gaps_over_3Ts > 0 || max(nearest{i}) > audit.Ts;
    if needs_check
        status(i) = "CHECK";
    else
        status(i) = "OK";
    end
end

timing_table = table(topics,messages,rate,median_dt,p95_dt,max_gap, ...
    duplicates,backward,gaps,p95_nearest,max_nearest,status, ...
    'VariableNames', {'Topic','Messages','MedianRate_Hz','MedianDt_ms', ...
    'P95Dt_ms','MaxGap_ms','Duplicates','BackwardJumps','GapsOver3Ts', ...
    'P95NearestRaw_ms','MaxNearestRaw_ms','Status'});
end

function signal_table = make_signal_table(data, names)
n = numel(names);
signal = strings(n,1); sample_count = zeros(n,1);
all_finite = false(n,1); minimum = zeros(n,1); maximum = zeros(n,1);
standard_deviation = zeros(n,1); p99_step = zeros(n,1);

for i = 1:n
    values = data.(names{i})(:);
    signal(i) = string(names{i});
    sample_count(i) = numel(values);
    all_finite(i) = all(isfinite(values));
    minimum(i) = min(values);
    maximum(i) = max(values);
    standard_deviation(i) = std(values);
    p99_step(i) = percentile_value(abs(diff(values)),99);
end

signal_table = table(signal,sample_count,all_finite,minimum,maximum, ...
    standard_deviation,p99_step, ...
    'VariableNames', {'Signal','SampleCount','AllFinite','Minimum', ...
    'Maximum','Std','P99AbsoluteStep'});
end

function [correlation, normalized_rmse, bias] = consistency_metrics(a, b)
a = a(:); b = b(:);
valid = isfinite(a) & isfinite(b);
a = a(valid); b = b(valid);
correlation = corr(a,b);
error = a-b;
normalized_rmse = sqrt(mean(error.^2)) / max(std(a),eps);
bias = mean(error);
end

function [best_correlation, best_lag_samples] = best_lag( ...
        input_signal, response_signal, max_lag_samples, use_absolute)
[~, correlations] = lag_curve(input_signal,response_signal, ...
    max_lag_samples,1);
if use_absolute
    score = abs(correlations);
else
    score = correlations;
end
[~, best_idx] = max(score,[],'omitnan');
lags = -max_lag_samples:max_lag_samples;
best_lag_samples = lags(best_idx);
best_correlation = correlations(best_idx);
end

function [lag_seconds, correlations] = lag_curve( ...
        input_signal, response_signal, max_lag_samples, Ts)
input_signal = input_signal(:);
response_signal = response_signal(:);
lags = -max_lag_samples:max_lag_samples;
correlations = nan(size(lags));

for i = 1:numel(lags)
    lag = lags(i);
    if lag >= 0
        input_part = input_signal(1:end-lag);
        response_part = response_signal(1+lag:end);
    else
        input_part = input_signal(1-lag:end);
        response_part = response_signal(1:end+lag);
    end
    if std(input_part) > eps && std(response_part) > eps
        correlations(i) = corr(input_part,response_part);
    end
end
lag_seconds = lags*Ts;
end

function value = percentile_value(x, percentage)
x = sort(x(isfinite(x)));
if isempty(x)
    value = NaN;
    return;
end
position = 1 + (numel(x)-1)*percentage/100;
lower_idx = floor(position);
upper_idx = ceil(position);
fraction = position-lower_idx;
value = x(lower_idx)*(1-fraction) + x(upper_idx)*fraction;
end

function plot_identity_line(x,y)
limits = [min([x(:);y(:)]), max([x(:);y(:)])];
plot(limits,limits,'--k','LineWidth',1);
end

function plot_lag_tile(axis_handle, lag_seconds, correlations, title_text)
plot(axis_handle,1000*lag_seconds,correlations,'LineWidth',1.2);
xline(axis_handle,0,'--');
xlabel(axis_handle,'Response lag [ms]');
ylabel(axis_handle,'Correlation');
title(axis_handle,title_text);
grid(axis_handle,'on');
end
