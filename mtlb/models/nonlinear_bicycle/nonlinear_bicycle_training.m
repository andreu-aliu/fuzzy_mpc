%% Identify nonlinear bicycle-model tyre parameters
% Fits front/rear Pacejka stiffness factors and peak axle forces using
% measured-steering rollouts. A one-step-only objective is deliberately
% avoided: at Ts = 20 ms it rewards an almost-static model and can produce
% parameters that look good for one step but diverge over an MPC horizon.

clear;
clc;

mtlb_dir = '/home/andreu/ros_ws/src/as/control/fuzzy_mpc/mtlb';
cd(mtlb_dir);
addpath(genpath(mtlb_dir));

%% Configuration

dataset_file = fullfile('data', 'datasets_training.mat');
min_vx = 2.0;                    % Ignore poorly conditioned low-speed data
fit_horizons = [1, 10, 30, 60]; % Include the current 60-step MPC horizon
max_windows_per_run = 12;        % Equal, bounded representation per run
filter_order = 3;
filter_window = 21;

if ~isfile(dataset_file)
    error('Training dataset not found: %s', dataset_file);
end

loaded = load(dataset_file, 'datasets', 'meta');
datasets = loaded.datasets;
Ts = loaded.meta.Ts;

%% Build balanced rollout windows

windows = struct('vy0', [], 'r0', [], 'vx', [], 'delta', [], ...
                 'vy_target', [], 'r_target', [], 'weight', []);
max_horizon = max(fit_horizons);

for run_idx = 1:numel(datasets)
    data = datasets(run_idx).data;

    ini = datasets(run_idx).ini;
    if isempty(ini)
        ini = 1;
    end

    fin = datasets(run_idx).fin;
    if isempty(fin)
        fin = numel(data.vx);
    end
    fin = min(fin, numel(data.vx));

    if fin - ini < max_horizon
        warning('Skipping run %d because it is shorter than %d steps.', ...
                run_idx, max_horizon);
        continue;
    end

    vy = sgolayfilt_custom(data.vy, filter_order, filter_window);
    r = sgolayfilt_custom(data.r, filter_order, filter_window);
    vx = sgolayfilt_custom(data.vx, filter_order, filter_window);
    delta = sgolayfilt_custom(data.delta, filter_order, filter_window);

    candidates = (ini:(fin - max_horizon))';
    valid = false(size(candidates));
    for candidate_idx = 1:numel(candidates)
        sequence = candidates(candidate_idx) + (0:max_horizon);
        valid(candidate_idx) = all(isfinite(vy(sequence))) && ...
            all(isfinite(r(sequence))) && ...
            all(isfinite(vx(sequence(1:end-1)))) && ...
            all(isfinite(delta(sequence(1:end-1)))) && ...
            all(abs(vx(sequence(1:end-1))) >= min_vx);
    end
    idx = candidates(valid);

    if numel(idx) > max_windows_per_run
        selection = unique(round(linspace(1, numel(idx), ...
                                          max_windows_per_run)));
        idx = idx(selection);
    end

    if isempty(idx)
        warning('Skipping run %d because it has no valid rollout windows.', ...
                run_idx);
        continue;
    end

    % Each run contributes equal total squared weight.
    run_weight = 1.0 / sqrt(numel(idx));

    for start_idx = idx(:)'
        windows.vy0(end + 1, 1) = vy(start_idx);
        windows.r0(end + 1, 1) = r(start_idx);
        windows.vx(end + 1, :) = vx(start_idx + (0:max_horizon-1));
        windows.delta(end + 1, :) = delta(start_idx + (0:max_horizon-1));
        windows.vy_target(end + 1, :) = vy(start_idx + fit_horizons);
        windows.r_target(end + 1, :) = r(start_idx + fit_horizons);
        windows.weight(end + 1, 1) = run_weight;
    end
end

if isempty(windows.vy0)
    error('No valid rollout windows were found in the training dataset.');
end

fprintf('Identification windows: %d from %d runs; horizons = %s steps.\n', ...
        numel(windows.vy0), numel(datasets), mat2str(fit_horizons));

%% Bounded nonlinear least-squares identification

params_nominal = nonlinear_bicycle_defaults();

% theta = [B_front, B_rear, D_front, D_rear]
theta_0 = [params_nominal.tire_Bf, params_nominal.tire_Br, ...
           params_nominal.tire_Df, params_nominal.tire_Dr];
lower_bound = [1.0, 1.0, 500.0, 500.0];
upper_bound = [30.0, 30.0, 6000.0, 6000.0];

vy_scale = max(std(windows.vy_target(:)), 1e-3);
r_scale = max(std(windows.r_target(:)), 1e-3);

residual_function = @(theta) prediction_residuals( ...
    theta, windows, params_nominal, Ts, fit_horizons, vy_scale, r_scale);

options = optimoptions('lsqnonlin', ...
    'Display', 'iter', ...
    'MaxFunctionEvaluations', 1000, ...
    'MaxIterations', 100, ...
    'FunctionTolerance', 1e-8, ...
    'StepTolerance', 1e-8);

[theta_fit, ~, residual, exitflag, output] = lsqnonlin( ...
    residual_function, theta_0, lower_bound, upper_bound, options);

[vy_nominal, r_nominal] = predict_rollouts( ...
    theta_0, windows, params_nominal, Ts, fit_horizons);
[vy_candidate, r_candidate] = predict_rollouts( ...
    theta_fit, windows, params_nominal, Ts, fit_horizons);

nominal_rmse = rollout_rmse(vy_nominal, r_nominal, windows);
candidate_rmse = rollout_rmse(vy_candidate, r_candidate, windows);
nominal_score = mean([nominal_rmse(end, 1) / vy_scale, ...
                      nominal_rmse(end, 2) / r_scale]);
candidate_score = mean([candidate_rmse(end, 1) / vy_scale, ...
                        candidate_rmse(end, 2) / r_scale]);
% A scalar average can hide a regression in one state. Since both vy and r
% are controller states, require improvement in each at the design horizon.
accepted = isfinite(candidate_score) && ...
           all(candidate_rmse(end, :) < nominal_rmse(end, :));
if accepted
    theta_selected = theta_fit;
    vy_selected = vy_candidate;
    r_selected = r_candidate;
else
    warning(['Rejecting fitted parameters: both 60-step state errors ' ...
             'must improve (mean normalized score nominal %.4g, ' ...
             'candidate %.4g).'], ...
            nominal_score, candidate_score);
    theta_selected = theta_0;
    vy_selected = vy_nominal;
    r_selected = r_nominal;
end

params = params_nominal;
params.tire_Bf = theta_selected(1);
params.tire_Br = theta_selected(2);
params.tire_Df = theta_selected(3);
params.tire_Dr = theta_selected(4);
params.trained_at = datetime('now');
params.training_dataset = string(dataset_file);
params.training_windows = numel(windows.vy0);
params.training_horizons = fit_horizons;

parameter_names = ["B front"; "B rear"; "D front [N]"; "D rear [N]"];
parameter_table = table(parameter_names, theta_0(:), theta_fit(:), ...
    theta_selected(:), 'VariableNames', ...
    {'Parameter', 'Nominal', 'Candidate', 'Selected'});
disp(parameter_table);

rmse_table = table(repelem(fit_horizons(:), 2), ...
    repmat(["vy [m/s]"; "r [rad/s]"], numel(fit_horizons), 1), ...
    reshape(nominal_rmse.', [], 1), reshape(candidate_rmse.', [], 1), ...
    'VariableNames', {'HorizonSteps', 'State', 'NominalRMSE', 'CandidateRMSE'});
disp(rmse_table);

fit_info.exitflag = exitflag;
fit_info.output = output;
fit_info.residual_norm = sum(residual.^2);
fit_info.nominal_rmse = nominal_rmse;
fit_info.candidate_rmse = candidate_rmse;
fit_info.accepted = accepted;
fit_info.fit_horizons = fit_horizons;

model_dir = fullfile(mtlb_dir, 'models', 'nonlinear_bicycle');
parameter_file = fullfile(model_dir, 'nonlinear_bicycle_params.mat');
save(parameter_file, 'params', 'fit_info');
clear nonlinear_bicycle;
fprintf('Saved fitted parameters to %s\n', parameter_file);

%% Fit diagnostics

identification_figure = figure('Name','Nonlinear bicycle identification');
tiledlayout(2, 2, 'TileSpacing', 'compact');

nexttile;
scatter(windows.vy_target(:, end), vy_selected(:, end), 8, '.', 'MarkerEdgeAlpha', 0.2);
hold on;
plot_identity(windows.vy_target(:, end));
xlabel('Measured v_y(k+60) [m/s]');
ylabel('Predicted v_y(k+60) [m/s]');
title('60-step lateral velocity');
grid on;
axis equal;

nexttile;
scatter(windows.r_target(:, end), r_selected(:, end), 8, '.', 'MarkerEdgeAlpha', 0.2);
hold on;
plot_identity(windows.r_target(:, end));
xlabel('Measured r(k+60) [rad/s]');
ylabel('Predicted r(k+60) [rad/s]');
title('60-step yaw rate');
grid on;
axis equal;

nexttile;
histogram(vy_selected(:, end) - windows.vy_target(:, end), 40);
xlabel('v_y prediction error [m/s]');
ylabel('Samples');
grid on;

nexttile;
histogram(r_selected(:, end) - windows.r_target(:, end), 40);
xlabel('r prediction error [rad/s]');
ylabel('Samples');
grid on;
save_script_figures('nonlinear_bicycle_training',identification_figure);

function residual = prediction_residuals(theta, windows, fixed, dt, ...
    fit_horizons, vy_scale, r_scale)
[vy_prediction, r_prediction] = predict_rollouts( ...
    theta, windows, fixed, dt, fit_horizons);
weights = windows.weight .* ones(1, numel(fit_horizons));
vy_error = weights .* (vy_prediction - windows.vy_target) / vy_scale;
r_error = weights .* (r_prediction - windows.r_target) / r_scale;
residual = [vy_error; r_error];
residual = residual(:);
end

function [vy_prediction, r_prediction] = predict_rollouts( ...
    theta, windows, p, dt, fit_horizons)
Bf = theta(1);
Br = theta(2);
Df = theta(3);
Dr = theta(4);
vy = windows.vy0;
r = windows.r0;
vy_prediction = zeros(numel(vy), numel(fit_horizons));
r_prediction = zeros(numel(r), numel(fit_horizons));
for step = 1:max(fit_horizons)
    vx = windows.vx(:, step);
    delta = windows.delta(:, step);
    vx_slip = max(abs(vx), p.min_vx);
    alpha_f = delta - atan2(vy + p.lf .* r, vx_slip);
    alpha_r = -atan2(vy - p.lr .* r, vx_slip);
    fyf = Df .* sin(p.tire_Cf .* atan(Bf .* alpha_f));
    fyr = Dr .* sin(p.tire_Cr .* atan(Br .* alpha_r));
    vy_dot = (fyf .* cos(delta) + fyr) ./ p.m - vx .* r;
    r_dot = (p.lf .* fyf .* cos(delta) - p.lr .* fyr) ./ p.Iz;
    vy = vy + dt .* vy_dot;
    r = r + dt .* r_dot;
    horizon_idx = find(fit_horizons == step, 1);
    if ~isempty(horizon_idx)
        vy_prediction(:, horizon_idx) = vy;
        r_prediction(:, horizon_idx) = r;
    end
end
end

function rmse = rollout_rmse(vy_prediction, r_prediction, windows)
rmse = [sqrt(mean((vy_prediction - windows.vy_target).^2, 1)).', ...
        sqrt(mean((r_prediction - windows.r_target).^2, 1)).'];
end

function plot_identity(values)
limits = [min(values), max(values)];
plot(limits, limits, 'k--', 'LineWidth', 1.0);
end
