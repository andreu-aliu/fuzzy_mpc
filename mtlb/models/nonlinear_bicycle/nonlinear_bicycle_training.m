%% Identify nonlinear bicycle-model tyre parameters
% Fits front/rear Pacejka stiffness factors and peak axle forces using
% measured one-step lateral-velocity and yaw-rate transitions.

clear;
clc;

mtlb_dir = '/home/andreu/ros_ws/src/as/control/fuzzy_mpc/mtlb';
cd(mtlb_dir);
addpath(genpath(mtlb_dir));

%% Configuration

dataset_file = fullfile('data', 'datasets_training.mat');
min_vx = 2.0;                    % Ignore poorly conditioned low-speed data
keep_factor = 2;                 % Initial temporal downsampling
max_samples_per_run = 3000;      % Prevent long runs dominating the fit
filter_order = 3;
filter_window = 21;

if ~isfile(dataset_file)
    error('Training dataset not found: %s', dataset_file);
end

loaded = load(dataset_file, 'datasets', 'meta');
datasets = loaded.datasets;
Ts = loaded.meta.Ts;

%% Build a balanced identification dataset

samples = struct('vy', [], 'r', [], 'vx', [], 'delta', [], 'mz', [], ...
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
    mz = sgolayfilt_custom(data.mz, filter_order, filter_window);

    idx = (ini:keep_factor:(fin - 1))';
    valid = isfinite(vy(idx)) & isfinite(vy(idx + 1)) & ...
            isfinite(r(idx)) & isfinite(r(idx + 1)) & ...
            isfinite(vx(idx)) & isfinite(delta(idx)) & ...
            isfinite(mz(idx)) & abs(vx(idx)) >= min_vx;
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

    % Each run contributes equal total weight, independently of its length.
    run_weight = 1.0 / sqrt(numel(idx));

    samples.vy = [samples.vy; vy(idx)]; %#ok<AGROW>
    samples.r = [samples.r; r(idx)]; %#ok<AGROW>
    samples.vx = [samples.vx; vx(idx)]; %#ok<AGROW>
    samples.delta = [samples.delta; delta(idx)]; %#ok<AGROW>
    samples.mz = [samples.mz; mz(idx)]; %#ok<AGROW>
    samples.vy_next = [samples.vy_next; vy(idx + 1)]; %#ok<AGROW>
    samples.r_next = [samples.r_next; r(idx + 1)]; %#ok<AGROW>
    samples.weight = [samples.weight; ...
        repmat(run_weight, numel(idx), 1)]; %#ok<AGROW>
end

if isempty(samples.vy)
    error('No valid samples were found in the training dataset.');
end

fprintf('Identification samples: %d from %d runs.\n', ...
        numel(samples.vy), numel(datasets));

%% Bounded nonlinear least-squares identification

params_nominal = nonlinear_bicycle_defaults();

% theta = [B_front, B_rear, D_front, D_rear]
theta_0 = [params_nominal.tire_Bf, params_nominal.tire_Br, ...
           params_nominal.tire_Df, params_nominal.tire_Dr];
lower_bound = [1.0, 1.0, 500.0, 500.0];
upper_bound = [30.0, 30.0, 6000.0, 6000.0];

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

params = params_nominal;
params.tire_Bf = theta_fit(1);
params.tire_Br = theta_fit(2);
params.tire_Df = theta_fit(3);
params.tire_Dr = theta_fit(4);
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

parameter_names = ["B front"; "B rear"; "D front [N]"; "D rear [N]"];
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

model_dir = fullfile(mtlb_dir, 'models', 'nonlinear_bicycle');
parameter_file = fullfile(model_dir, 'nonlinear_bicycle_params.mat');
save(parameter_file, 'params', 'fit_info');
clear nonlinear_bicycle;
fprintf('Saved fitted parameters to %s\n', parameter_file);

%% Fit diagnostics

figure('Name', 'Nonlinear bicycle identification');
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

function [vy_next, r_next] = predict_lateral(theta, samples, p, dt)
Bf = theta(1);
Br = theta(2);
Df = theta(3);
Dr = theta(4);

vx_slip = max(abs(samples.vx), p.min_vx);
alpha_f = samples.delta - ...
          atan2(samples.vy + p.lf .* samples.r, vx_slip);
alpha_r = -atan2(samples.vy - p.lr .* samples.r, vx_slip);

fyf = Df .* sin(p.tire_Cf .* atan(Bf .* alpha_f));
fyr = Dr .* sin(p.tire_Cr .* atan(Br .* alpha_r));

vy_dot = (fyf .* cos(samples.delta) + fyr) ./ p.m - ...
         samples.vx .* samples.r;
r_dot = (p.lf .* fyf .* cos(samples.delta) - p.lr .* fyr + ...
         samples.mz) ./ p.Iz;

vy_next = samples.vy + dt .* vy_dot;
r_next = samples.r + dt .* r_dot;
end

function plot_identity(values)
limits = [min(values), max(values)];
plot(limits, limits, 'k--', 'LineWidth', 1.0);
end
