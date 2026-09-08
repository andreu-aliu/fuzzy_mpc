%% Identify nonlinear double-track tyre parameters
% Fits front/rear Pacejka stiffness factors and friction coefficients using
% measured one-step lateral-velocity and yaw-rate transitions.

clear;
clc;

mtlb_dir = '/home/andreu/ros_ws/src/as/control/fuzzy_mpc/mtlb';
cd(mtlb_dir);
addpath(genpath(mtlb_dir));

%% Configuration

dataset_file = fullfile('data', 'datasets_training.mat');
min_vx = 2.0;
keep_factor = 2;
max_samples_per_run = 3000;
filter_order = 3;
filter_window = 21;

if ~isfile(dataset_file)
    error('Training dataset not found: %s', dataset_file);
end

loaded = load(dataset_file, 'datasets', 'meta');
datasets = loaded.datasets;
Ts = loaded.meta.Ts;

%% Build a balanced identification dataset

samples = struct('vy', [], 'r', [], 'vx', [], 'delta', [], ...
                 'vy_next', [], 'r_next', [], 'weight', []);

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

    if fin <= ini
        warning('Skipping run %d because it has fewer than two samples.', run_idx);
        continue;
    end

    vy = sgolayfilt_custom(data.vy, filter_order, filter_window);
    r = sgolayfilt_custom(data.r, filter_order, filter_window);
    vx = sgolayfilt_custom(data.vx, filter_order, filter_window);
    delta = sgolayfilt_custom(data.delta, filter_order, filter_window);

    idx = (ini:keep_factor:(fin - 1))';
    valid = isfinite(vy(idx)) & isfinite(vy(idx + 1)) & ...
            isfinite(r(idx)) & isfinite(r(idx + 1)) & ...
            isfinite(vx(idx)) & isfinite(delta(idx)) & ...
            vx(idx) >= min_vx;
    idx = idx(valid);

    if numel(idx) > max_samples_per_run
        selection = round(linspace(1, numel(idx), max_samples_per_run));
        idx = idx(selection);
    end

    if isempty(idx)
        warning('Skipping run %d because it has no valid dynamic samples.', ...
                run_idx);
        continue;
    end

    run_weight = 1.0 / sqrt(numel(idx));

    samples.vy = [samples.vy; vy(idx)];
    samples.r = [samples.r; r(idx)];
    samples.vx = [samples.vx; vx(idx)];
    samples.delta = [samples.delta; delta(idx)];
    samples.vy_next = [samples.vy_next; vy(idx + 1)];
    samples.r_next = [samples.r_next; r(idx + 1)];
    samples.weight = [samples.weight; ...
        repmat(run_weight, numel(idx), 1)];
end

if isempty(samples.vy)
    error('No valid samples were found in the training dataset.');
end

fprintf('Identification samples: %d from %d runs.\n', ...
        numel(samples.vy), numel(datasets));

%% Bounded nonlinear least-squares identification

params_nominal = nonlinear_double_track_defaults();

% theta = [B_front, B_rear, mu_front, mu_rear]
theta_0 = [params_nominal.tire_Bf, params_nominal.tire_Br, ...
           params_nominal.mu_front, params_nominal.mu_rear];
lower_bound = [1.0, 1.0, 0.3, 0.3];
upper_bound = [30.0, 30.0, 2.5, 2.5];

vy_scale = max(std(samples.vy_next), 1e-3);
r_scale = max(std(samples.r_next), 1e-3);

residual_function = @(theta) prediction_residuals( ...
    theta, samples, params_nominal, Ts, vy_scale, r_scale);

options = optimoptions('lsqnonlin', ...
    'Display', 'iter', ...
    'MaxFunctionEvaluations', 1000, ...
    'MaxIterations', 100, ...
    'FunctionTolerance', 1e-8, ...
    'StepTolerance', 1e-8);

[theta_fit, ~, residual, exitflag, output] = lsqnonlin( ...
    residual_function, theta_0, lower_bound, upper_bound, options);

params = apply_parameters(params_nominal, theta_fit);
params.trained_at = datetime('now');
params.training_dataset = string(dataset_file);
params.training_samples = numel(samples.vy);

[vy_nominal, r_nominal] = predict_lateral( ...
    theta_0, samples, params_nominal, Ts);
[vy_fitted, r_fitted] = predict_lateral( ...
    theta_fit, samples, params_nominal, Ts);

nominal_rmse = [sqrt(mean((vy_nominal - samples.vy_next).^2)), ...
                sqrt(mean((r_nominal - samples.r_next).^2))];
fitted_rmse = [sqrt(mean((vy_fitted - samples.vy_next).^2)), ...
               sqrt(mean((r_fitted - samples.r_next).^2))];

parameter_names = ["B front"; "B rear"; "mu front"; "mu rear"];
parameter_table = table(parameter_names, theta_0(:), theta_fit(:), ...
    'VariableNames', {'Parameter', 'Nominal', 'Fitted'});
disp(parameter_table);

rmse_table = table(["vy [m/s]"; "r [rad/s]"], ...
    nominal_rmse(:), fitted_rmse(:), ...
    'VariableNames', {'State', 'NominalRMSE', 'FittedRMSE'});
disp(rmse_table);

fit_info.exitflag = exitflag;
fit_info.output = output;
fit_info.residual_norm = sum(residual.^2);
fit_info.nominal_rmse = nominal_rmse;
fit_info.fitted_rmse = fitted_rmse;

model_dir = fullfile(mtlb_dir, 'models', 'nonlinear_double_track');
parameter_file = fullfile(model_dir, 'nonlinear_double_track_params.mat');
save(parameter_file, 'params', 'fit_info');
clear nonlinear_double_track;
fprintf('Saved fitted parameters to %s\n', parameter_file);

%% Fit diagnostics

figure('Name', 'Nonlinear double-track identification');
tiledlayout(2, 2, 'TileSpacing', 'compact');

nexttile;
scatter(samples.vy_next, vy_fitted, 5, '.', 'MarkerEdgeAlpha', 0.15);
hold on;
plot_identity(samples.vy_next);
xlabel('Measured v_y(k+1) [m/s]');
ylabel('Predicted v_y(k+1) [m/s]');
title('Lateral velocity');
grid on;
axis equal;

nexttile;
scatter(samples.r_next, r_fitted, 5, '.', 'MarkerEdgeAlpha', 0.15);
hold on;
plot_identity(samples.r_next);
xlabel('Measured r(k+1) [rad/s]');
ylabel('Predicted r(k+1) [rad/s]');
title('Yaw rate');
grid on;
axis equal;

nexttile;
histogram(vy_fitted - samples.vy_next, 80);
xlabel('v_y prediction error [m/s]');
ylabel('Samples');
grid on;

nexttile;
histogram(r_fitted - samples.r_next, 80);
xlabel('r prediction error [rad/s]');
ylabel('Samples');
grid on;

function residual = prediction_residuals( ...
    theta, samples, fixed, dt, vy_scale, r_scale)
[vy_prediction, r_prediction] = predict_lateral(theta, samples, fixed, dt);
vy_error = samples.weight .* ...
           (vy_prediction - samples.vy_next) / vy_scale;
r_error = samples.weight .* ...
          (r_prediction - samples.r_next) / r_scale;
residual = [vy_error; r_error];
end

function [vy_next, r_next] = predict_lateral(theta, samples, fixed, dt)
p = apply_parameters(fixed, theta);
[vy_dot, r_dot] = nonlinear_double_track_dynamics( ...
    samples.vy, samples.r, samples.vx, samples.delta, p);
vy_next = samples.vy + dt .* vy_dot;
r_next = samples.r + dt .* r_dot;
end

function p = apply_parameters(p, theta)
p.tire_Bf = theta(1);
p.tire_Br = theta(2);
p.mu_front = theta(3);
p.mu_rear = theta(4);
end

function plot_identity(values)
limits = [min(values), max(values)];
plot(limits, limits, 'k--', 'LineWidth', 1.0);
end
