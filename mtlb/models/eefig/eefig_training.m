%% Train the lateral EEFig model offline and inspect held-out performance
% Learned state: [vy; r]
% Scheduling/input vector: [vx; delta]
% Target: [vy(k+1); r(k+1)]

clear;
clc;

mtlb_dir = '/home/andreu/ros_ws/src/as/control/fuzzy_mpc/mtlb';
cd(mtlb_dir);
addpath(genpath(mtlb_dir));

%% Configuration

training_dataset_file = fullfile('data', 'datasets_training.mat');
evaluation_dataset_file = fullfile('data', 'datasets_evaluation.mat');
model_file = fullfile('models', 'eefig', 'eefig.mat');

keep_factor = 1;       % Use every transition; preserves stream continuity.
filter_order = 3;
filter_window = 21;

params = struct();
params.phi = 100;      % 2 seconds at Ts = 0.02 s.
params.min_initial_samples = params.phi;
params.n_anomaly_max = 5;
params.rls_forgetting = 0.99;
params.rls_mode = 'per_granule';
params.rls_P0 = 10;
params.lambda = 3;
params.iota = 0.3;
params.confidence = 0.999;
params.c_separation = 0.5; % Tuned structural balance for the real dataset.
params.use_c_separation = true;
params.use_pjg_quality_check = true;
params.betaS = -inf;   % Vehicle premise variables are signed.
params.betaT = inf;
params.reg = 1e-8;
params.debug = false;

%% Prepare independent, run-separated transitions

prepared = prepare_eefig_data(training_dataset_file, ...
    evaluation_dataset_file, keep_factor, filter_order, filter_window);

learner = EEFIGLearning(2, 2, params);
training_sample_count = sum([prepared.training_runs.sample_count]);
history.sample = (1:training_sample_count)';
history.run = zeros(training_sample_count, 1);
history.granules = zeros(training_sample_count, 1);
history.anomaly = false(training_sample_count, 1);
history.created = false(training_sample_count, 1);
history.reverted_updates = zeros(training_sample_count, 1);
history.active_granule = nan(training_sample_count, 1);

%% Sequential offline learning across all training runs

cursor = 0;
for run_idx = 1:numel(prepared.training_runs)
    run = prepared.training_runs(run_idx);
    learner.startNewRun();

    fprintf('Training run %d/%d: %s | %s | %d transitions\n', ...
        run_idx, numel(prepared.training_runs), run.event, ...
        run.track_layout, run.sample_count);

    for sample_idx = 1:run.sample_count
        status = learner.updatePair([run.Xn(:, sample_idx); ...
            run.Un(:, sample_idx)], run.Xnextn(:, sample_idx), 'offline');

        cursor = cursor + 1;
        history.run(cursor) = run_idx;
        history.granules(cursor) = learner.NG;
        history.anomaly(cursor) = status.is_anomaly;
        history.created(cursor) = status.created_new_granule;
        history.reverted_updates(cursor) = numel(status.reverted_idx);
        history.active_granule(cursor) = status.active_idx;
    end
end

if learner.NG == 0
    error('EEFIG training ended without creating an initial granule.');
end

%% Frozen one-step evaluation (no adaptation and no parameter updates)

training_fit = evaluateFrozen(learner, prepared.training_runs, ...
    prepared.scale_x, "Training");
evaluation = evaluateFrozen(learner, prepared.evaluation_runs, ...
    prepared.scale_x, "Held-out evaluation");

% The saved artifact is a trained prior, not a continuation of the final
% training run. Clear only transient windows/anomaly state before saving;
% learned granules and consequents are preserved.
learner.startNewRun();

%% Store the offline model and reproducibility metadata

eefig_model = struct();
eefig_model.learner = learner;
eefig_model.Ts = prepared.Ts;
eefig_model.norm.scale_x = prepared.scale_x;
eefig_model.norm.scale_u = prepared.scale_u;
eefig_model.norm.training_min = prepared.training_min;
eefig_model.norm.training_max = prepared.training_max;
eefig_model.training.dataset_file = string(training_dataset_file);
eefig_model.training.evaluation_dataset_file = string(evaluation_dataset_file);
eefig_model.training.keep_factor = keep_factor;
eefig_model.training.filter_order = filter_order;
eefig_model.training.filter_window = filter_window;
eefig_model.training.params = params;
eefig_model.training.run_count = numel(prepared.training_runs);
eefig_model.training.sample_count = training_sample_count;
eefig_model.training.granule_count = learner.NG;
eefig_model.training.created_at = datetime('now');
eefig_model.training.frozen_fit_per_run = training_fit.per_run;
eefig_model.training.frozen_fit_overall = training_fit.overall;
eefig_model.frozen_evaluation.per_run = evaluation.per_run;
eefig_model.frozen_evaluation.overall = evaluation.overall;

save(model_file, 'eefig_model', '-v7.3');
eefig_reset();
fprintf('Saved frozen offline EEFig model to %s\n', model_file);

%% Model insights

eefig_model_insights(learner, history, evaluation, prepared);

function evaluation = evaluateFrozen(learner, runs, scale_x, split_label)
evaluation.runs = repmat(struct('measured',[],'predicted',[], ...
    'error',[],'active_granule',[],'outside_all_granules',[]), ...
    numel(runs), 1);
per_run_name = strings(numel(runs), 1);
rmse_vy = zeros(numel(runs), 1);
rmse_r = zeros(numel(runs), 1);
mae_vy = zeros(numel(runs), 1);
mae_r = zeros(numel(runs), 1);
outside_fraction = zeros(numel(runs), 1);
all_errors = [];
all_outside = [];

for run_idx = 1:numel(runs)
    run = runs(run_idx);
    predicted_n = zeros(2, run.sample_count);
    active = zeros(run.sample_count, 1);
    outside = false(run.sample_count, 1);

    for sample_idx = 1:run.sample_count
        predicted_n(:, sample_idx) = learner.predict( ...
            run.Xn(:, sample_idx), run.Un(:, sample_idx));
        zeta = [run.Xn(:, sample_idx); run.Un(:, sample_idx)];
        [g, ~] = learner.computeMemberships(zeta);
        [~, active(sample_idx)] = max(g);
        admitted = false;
        for granule_idx = 1:learner.NG
            admitted = admitted || learner.granules{granule_idx}.contains( ...
                zeta, learner.epsilon);
        end
        outside(sample_idx) = ~admitted;
    end

    predicted = predicted_n .* scale_x;
    errors = predicted - run.Xnext;
    evaluation.runs(run_idx).measured = run.Xnext;
    evaluation.runs(run_idx).predicted = predicted;
    evaluation.runs(run_idx).error = errors;
    evaluation.runs(run_idx).active_granule = active;
    evaluation.runs(run_idx).outside_all_granules = outside;

    per_run_name(run_idx) = sprintf('%d: %s / %s', run_idx, ...
        run.event, run.track_layout);
    rmse_vy(run_idx) = sqrt(mean(errors(1, :).^2));
    rmse_r(run_idx) = sqrt(mean(errors(2, :).^2));
    mae_vy(run_idx) = mean(abs(errors(1, :)));
    mae_r(run_idx) = mean(abs(errors(2, :)));
    outside_fraction(run_idx) = mean(outside);
    all_errors = [all_errors, errors]; %#ok<AGROW>
    all_outside = [all_outside; outside]; %#ok<AGROW>
end

evaluation.per_run = table(per_run_name, rmse_vy, rmse_r, mae_vy, mae_r, ...
    outside_fraction, 'VariableNames', ...
    {'Run','RMSE_vy','RMSE_r','MAE_vy','MAE_r','OutsideFraction'});
evaluation.overall = table( ...
    sqrt(mean(all_errors(1, :).^2)), ...
    sqrt(mean(all_errors(2, :).^2)), ...
    mean(abs(all_errors(1, :))), ...
    mean(abs(all_errors(2, :))), ...
    prctile(abs(all_errors(1, :)), 95), ...
    prctile(abs(all_errors(2, :)), 95), ...
    mean(all_outside), ...
    'VariableNames', {'RMSE_vy','RMSE_r','MAE_vy','MAE_r', ...
    'P95_vy','P95_r','OutsideFraction'});

fprintf('\nFrozen %s one-step performance:\n', split_label);
disp(evaluation.overall);
disp(evaluation.per_run);
end
