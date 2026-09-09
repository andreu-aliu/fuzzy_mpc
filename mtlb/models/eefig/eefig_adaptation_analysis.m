%% Compare frozen and online-adaptive EEFig behavior on held-out runs
% The saved offline model is reloaded before every independent experiment.
% Online errors are always recorded before the measured transition is used.

clear;
clc;

mtlb_dir = '/home/andreu/ros_ws/src/as/control/fuzzy_mpc/mtlb';
cd(mtlb_dir);
addpath(genpath(mtlb_dir));

%% Configuration

dataset_file = fullfile('data', 'datasets_evaluation.mat');
propagation_run = 1;
propagation_start = 400;
propagation_horizon = 150;

if ~isfile(dataset_file)
    error('Evaluation dataset not found: %s', dataset_file);
end

source = load(dataset_file, 'datasets', 'meta');
Ts = source.meta.Ts;
runs = buildEvaluationRuns(source.datasets, Ts);

%% Frozen versus adaptive one-step evaluation

frozen_errors = [];
adaptive_errors = [];
adaptive_creations = zeros(numel(runs), 1);

for run_idx = 1:numel(runs)
    run = runs(run_idx);

    % Frozen baseline.
    eefig_reset();
    for k = 1:run.N-1
        prediction = eefig(run.X(:, k), run.U(:, k), Ts);
        frozen_errors(:, end + 1) = ...
            prediction([2, 4]) - run.X([2, 4], k + 1); %#ok<SAGROW>
    end

    % Prequential evaluation: predict, score, then update.
    eefig_reset();
    for k = 1:run.N-1
        prediction = eefig(run.X(:, k), run.U(:, k), Ts);
        adaptive_errors(:, end + 1) = ...
            prediction([2, 4]) - run.X([2, 4], k + 1); %#ok<SAGROW>
        status = eefig_online_update( ...
            run.X(:, k), run.U(:, k), run.X(:, k + 1));
        adaptive_creations(run_idx) = adaptive_creations(run_idx) + ...
            status.created_new_granule;
    end
end

one_step_table = errorTable(frozen_errors, adaptive_errors, ...
    "Frozen offline", "Predict-then-update");
fprintf('\nHeld-out one-step comparison:\n');
disp(one_step_table);
fprintf('New granules created per adaptive evaluation run:\n');
disp(table((1:numel(runs))', adaptive_creations, ...
    'VariableNames', {'Run','NewGranules'}));

%% Frozen versus continuously adaptive propagation

validateattributes(propagation_run, {'numeric'}, ...
    {'scalar','integer','>=',1,'<=',numel(runs)});
run = runs(propagation_run);
last_index = min(propagation_start + propagation_horizon, run.N);
indices = propagation_start:last_index;
if numel(indices) < 2
    error('The selected propagation interval must contain two samples.');
end

frozen_states = zeros(6, numel(indices));
adaptive_states = zeros(6, numel(indices));
frozen_states(:, 1) = run.X(:, indices(1));
adaptive_states(:, 1) = run.X(:, indices(1));

eefig_reset();
for q = 1:numel(indices)-1
    k = indices(q);
    frozen_states(:, q + 1) = eefig( ...
        frozen_states(:, q), run.U(:, k), Ts);
end

eefig_reset();
adaptive_created = 0;
for q = 1:numel(indices)-1
    k = indices(q);

    % Propagate first, without access to the next measurement.
    adaptive_states(:, q + 1) = eefig( ...
        adaptive_states(:, q), run.U(:, k), Ts);

    % Once x(k+1) has been observed, adapt for future predictions. The
    % propagated state is deliberately not reset to the measurement.
    status = eefig_online_update( ...
        run.X(:, k), run.U(:, k), run.X(:, k + 1));
    adaptive_created = adaptive_created + status.created_new_granule;
end
eefig_reset();

measured = run.X(:, indices);
frozen_propagation_error = frozen_states([2, 4], :) - measured([2, 4], :);
adaptive_propagation_error = adaptive_states([2, 4], :) - measured([2, 4], :);
propagation_table = errorTable(frozen_propagation_error, ...
    adaptive_propagation_error, "Frozen propagation", ...
    "Online-adaptive propagation");

fprintf('\nPropagation comparison, evaluation run %d:\n', propagation_run);
disp(propagation_table);
fprintf('New granules during adaptive propagation: %d\n', adaptive_created);

%% Plots

t = (0:numel(indices)-1) * Ts;
figure('Name', 'EEFIG frozen versus adaptive propagation');
tiledlayout(2, 1, 'TileSpacing', 'compact');

nexttile;
plot(t, measured(2, :), 'k', 'LineWidth', 1.8, ...
    'DisplayName', 'Measured');
hold on;
plot(t, frozen_states(2, :), 'LineWidth', 1.3, ...
    'DisplayName', 'Frozen');
plot(t, adaptive_states(2, :), 'LineWidth', 1.3, ...
    'DisplayName', 'Adaptive');
hold off;
ylabel('v_y [m/s]');
legend('Location', 'best');
grid on;

nexttile;
plot(t, measured(4, :), 'k', 'LineWidth', 1.8, ...
    'DisplayName', 'Measured');
hold on;
plot(t, frozen_states(4, :), 'LineWidth', 1.3, ...
    'DisplayName', 'Frozen');
plot(t, adaptive_states(4, :), 'LineWidth', 1.3, ...
    'DisplayName', 'Adaptive');
hold off;
xlabel('Time [s]');
ylabel('r [rad/s]');
legend('Location', 'best');
grid on;

function runs = buildEvaluationRuns(datasets, Ts)
runs = repmat(struct('X',[],'U',[],'N',0), numel(datasets), 1);
for run_idx = 1:numel(datasets)
    data = datasets(run_idx).data;
    ini = datasets(run_idx).ini;
    fin = datasets(run_idx).fin;
    if isempty(ini), ini = 1; end
    if isempty(fin), fin = numel(data.vx); end
    fin = min(fin, numel(data.vx));
    idx = ini:fin;

    [y_local, psi_local] = localFrame(data.x(idx), data.y(idx));
    delta = data.delta(idx);
    delta_dot = gradient(delta, Ts);
    runs(run_idx).X = [y_local'; data.vy(idx)'; psi_local'; ...
        data.r(idx)'; delta'; delta_dot'];
    runs(run_idx).U = [data.vx(idx)'; data.st(idx)'];
    runs(run_idx).N = numel(idx);
end
end

function [y_local, psi_local] = localFrame(x, y)
x = x(:);
y = y(:);
dx = gradient(x);
dy = gradient(y);
psi_global = unwrap(atan2(dy, dx));
psi0 = psi_global(1);
y_local = -(x - x(1)) * sin(psi0) + (y - y(1)) * cos(psi0);
psi_local = unwrap(psi_global - psi0);
end

function result = errorTable(error_a, error_b, name_a, name_b)
method = [name_a; name_b];
rmse_vy = [sqrt(mean(error_a(1, :).^2)); ...
    sqrt(mean(error_b(1, :).^2))];
rmse_r = [sqrt(mean(error_a(2, :).^2)); ...
    sqrt(mean(error_b(2, :).^2))];
mae_vy = [mean(abs(error_a(1, :))); mean(abs(error_b(1, :)))];
mae_r = [mean(abs(error_a(2, :))); mean(abs(error_b(2, :)))];
result = table(method, rmse_vy, rmse_r, mae_vy, mae_r, ...
    'VariableNames', {'Method','RMSE_vy','RMSE_r','MAE_vy','MAE_r'});
end
