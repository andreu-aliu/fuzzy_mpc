%% Held-out comparison of lateral vehicle-dynamics models
% Primary scope: predict and recursively propagate [vy, r] from measured
% [vx, delta]. Steering-command/actuator behavior is reported separately.

clear;
clc;

mtlb_dir = '/home/andreu/ros_ws/src/as/control/fuzzy_mpc/mtlb';
cd(mtlb_dir);
addpath(genpath(mtlb_dir));

%% Configuration

Ts = 0.02;
dataset_file = fullfile('data', 'datasets_evaluation.mat');

% The 60-step / 1.2-s horizon is the current MPC prediction horizon.
cfg.horizons = [1, 5, 10, 20, 40, 60];
cfg.focus_horizon = 60;
cfg.rolling_stride = 20;       % one rollout origin every 0.4 s
cfg.min_bin_samples = 50;
cfg.suppress_model_prints = true;
cfg.run_end_to_end = true;
cfg.run_adaptive_eefig = true;
cfg.plot_selected_window = true;
cfg.selected_run = 1;          % index in datasets_evaluation.mat
cfg.selected_start = 1001;     % sample inside the selected run
cfg.selected_horizon = 300;
cfg.divergence_abs_vy = 10;  % numerical-failure diagnostic, not clipping
cfg.divergence_abs_r = 5;
cfg.plot_outlier_factor = 10; % axis-only robust clipping
cfg.vx_edges = [0:2:30, inf];
cfg.delta_edges = [0:0.02:0.40, inf];

models = {
    @anfis_direct,           'ANFIS direct',           [0.000, 0.447, 0.741];
    @anfis_delta,            'ANFIS delta',            [0.850, 0.325, 0.098];
    @anfis_dot,              'ANFIS derivative',       [0.929, 0.694, 0.125];
    @anfis_residuals,        'ANFIS LTV residual',     [0.494, 0.184, 0.556];
    @eefig,                  'EEFIG offline',          [0.466, 0.674, 0.188];
    @nonlinear_bicycle,      'Nonlinear bicycle',      [0.301, 0.745, 0.933];
    @nonlinear_double_track, 'Nonlinear double track', [0.635, 0.078, 0.184];
    @ltv,                    'LTV MPC',                [0.650, 0.650, 0.650];
};

baseline_name = 'Persistence / constant yaw rate';
baseline_color = [0.950, 0.450, 0.750];
model_names = [string(models(:,2)); string(baseline_name)];
model_colors = [vertcat(models{:,3}); baseline_color];
adaptive_color = [0.000, 0.620, 0.500];

validateattributes(cfg.horizons, {'numeric'}, ...
    {'vector','integer','positive','increasing'});
if ~ismember(cfg.focus_horizon, cfg.horizons)
    error('focus_horizon must be included in cfg.horizons.');
end

%% Load and validate complete held-out runs

if ~isfile(dataset_file)
    error(['Evaluation dataset not found: %s\nRun prepare_datasets.m ' ...
        'before comparing models.'], dataset_file);
end

source = load(dataset_file, 'datasets', 'meta');
if abs(source.meta.Ts - Ts) > eps(max(source.meta.Ts, Ts))
    error('Dataset Ts (%g s) does not match model Ts (%g s).', ...
        source.meta.Ts, Ts);
end

runs = buildRuns(source.datasets, Ts);
heading_quality = headingQualityTable(runs, Ts);
fprintf('\nRecorded-heading consistency (d(psi)/dt versus r):\n');
disp(heading_quality);
if any(~isfinite(heading_quality.Correlation) | ...
        heading_quality.Correlation < 0.8 | ...
        heading_quality.RMSE_rate > 0.15)
    warning(['At least one run has weak heading/yaw-rate consistency. ' ...
        'Inspect visualize_data.m before interpreting heading errors.']);
end

fprintf(['\nComparison configuration: %d runs, horizons [%s] steps, ' ...
    'rolling stride %d steps.\n'], numel(runs), ...
    strjoin(string(cfg.horizons), ', '), cfg.rolling_stride);
fprintf('Primary MPC horizon: %d steps = %.2f s.\n', ...
    cfg.focus_horizon, cfg.focus_horizon * Ts);

eefig_reset();

%% One-step prediction from measured states and measured steering

one_step = evaluateOneStep(models, model_names, runs, Ts, ...
    cfg.suppress_model_prints);
one_step.summary = summarizeOneStep(one_step, runs, model_names);

fprintf('\nONE-STEP SUMMARY\n');
fprintf(['Sample = every transition weighted equally. Run = every run ' ...
    'weighted equally. Lap = run RMSE weighted by declared lap count.\n']);
disp(one_step.summary);

fprintf('\nOne-step RMSE by event:\n');
disp(groupedOneStepTable(one_step, runs, model_names, 'event'));
fprintf('\nOne-step RMSE by track layout:\n');
disp(groupedOneStepTable(one_step, runs, model_names, 'track_layout'));

plotOneStepSummary(one_step.summary, model_names, model_colors, cfg);
plotOneStepResiduals(one_step, model_names, model_colors, cfg);
plotPerRunOneStep(one_step, runs, model_names, model_colors, cfg);

%% Systematic dynamics-only rolling propagation

fprintf('\nRunning dynamics-only rolling propagation...\n');
dynamics = evaluateRollingPropagation(models, model_names, runs, Ts, cfg, ...
    'measured_delta');
dynamics.summary = summarizePropagation(dynamics, runs, model_names, Ts);
dynamics.divergence = propagationDivergenceTable(dynamics.summary);
ltv_stability = ltvEulerStabilityDiagnostic(runs, Ts);

focus_dynamics = focusHorizonTable(dynamics.summary, cfg.focus_horizon);
fprintf('\nDYNAMICS-ONLY PROPAGATION AT %d STEPS (%.2f s)\n', ...
    cfg.focus_horizon, cfg.focus_horizon * Ts);
disp(focus_dynamics);
fprintf('\nNumerical propagation failures (full values remain in tables):\n');
disp(dynamics.divergence);
fprintf('\nLTV forward-Euler lateral stability diagnostic:\n');
disp(ltv_stability.spectral_radius);
fprintf(['Maximum spectral radius on the 0.1-30 m/s diagnostic grid: ' ...
    '%.4f. Evaluation samples using the 3 m/s clamp: %.2f%%.\n'], ...
    ltv_stability.maximum_euler_radius, ...
    100*ltv_stability.evaluation_fraction_clamped);

fprintf('\nDynamics-only RMSE at the MPC horizon by event:\n');
disp(groupedPropagationTable(dynamics, runs, model_names, ...
    cfg.focus_horizon, 'event'));
fprintf('\nDynamics-only RMSE at the MPC horizon by track layout:\n');
disp(groupedPropagationTable(dynamics, runs, model_names, ...
    cfg.focus_horizon, 'track_layout'));

plotPropagationCurves(dynamics.summary, model_names, model_colors, ...
    cfg.focus_horizon, 'Dynamics-only propagation', cfg);
plotFocusAggregation(focus_dynamics, model_names, model_colors, ...
    'Dynamics-only propagation at MPC horizon', cfg);

%% Separate steering-command/actuator diagnostic

end_to_end = struct();
if cfg.run_end_to_end
    fprintf('\nRunning separate end-to-end actuator diagnostic...\n');
    end_to_end = evaluateRollingPropagation(models, model_names, runs, ...
        Ts, cfg, 'steering_command');
    end_to_end.summary = summarizePropagation( ...
        end_to_end, runs, model_names, Ts);
    end_to_end.divergence = propagationDivergenceTable( ...
        end_to_end.summary);
    focus_end_to_end = focusHorizonTable( ...
        end_to_end.summary, cfg.focus_horizon);

    fprintf(['\nEND-TO-END DIAGNOSTIC AT %d STEPS (%.2f s)\n' ...
        'This table is not the primary vehicle-dynamics ranking.\n'], ...
        cfg.focus_horizon, cfg.focus_horizon * Ts);
    disp(focus_end_to_end);
    fprintf('\nEnd-to-end numerical propagation failures:\n');
    disp(end_to_end.divergence);
    plotPropagationCurves(end_to_end.summary, model_names, model_colors, ...
        cfg.focus_horizon, 'End-to-end steering-actuator diagnostic', cfg);
    plotSteeringActuatorDiagnostic(end_to_end.summary, model_names, ...
        model_colors, cfg.focus_horizon, cfg);
end

%% Frozen versus causally adaptive EEFig

eefig_adaptive = struct();
if cfg.run_adaptive_eefig
    fprintf('\nRunning causal EEFig adaptation analysis...\n');
    eefig_adaptive = evaluateAdaptiveEefig(runs, Ts, cfg);
    eefig_adaptive.summary = summarizePropagation( ...
        eefig_adaptive, runs, "EEFIG adaptive", Ts);
    eefig_adaptive.one_step_summary = summarizeOneStep( ...
        eefig_adaptive.one_step, runs, "EEFIG adaptive");

    frozen_idx = find(model_names == "EEFIG offline", 1);
    baseline_idx = find(model_names == ...
        "Persistence / constant yaw rate", 1);
    eefig_adaptive.one_step_summary. ...
        SkillVsPersistence_vy_percent = 100*( ...
        one_step.summary.SampleRMSE_vy(baseline_idx) ...
        - eefig_adaptive.one_step_summary.SampleRMSE_vy) ...
        / one_step.summary.SampleRMSE_vy(baseline_idx);
    eefig_adaptive.one_step_summary. ...
        SkillVsPersistence_r_percent = 100*( ...
        one_step.summary.SampleRMSE_r(baseline_idx) ...
        - eefig_adaptive.one_step_summary.SampleRMSE_r) ...
        / one_step.summary.SampleRMSE_r(baseline_idx);
    eefig_adaptive.summary = attachPropagationBaselineSkill( ...
        eefig_adaptive.summary, dynamics.summary);
    frozen_focus = focus_dynamics(focus_dynamics.Model == ...
        "EEFIG offline", :);
    adaptive_focus = focusHorizonTable( ...
        eefig_adaptive.summary, cfg.focus_horizon);

    fprintf('\nFROZEN VERSUS ADAPTIVE EEFIG: ONE STEP\n');
    disp([one_step.summary(frozen_idx,:); ...
        eefig_adaptive.one_step_summary]);
    fprintf('\nFROZEN VERSUS ADAPTIVE EEFIG: MPC HORIZON\n');
    disp([frozen_focus; adaptive_focus]);
    fprintf('New granules created independently in each evaluation run:\n');
    disp(table((1:numel(runs))', string({runs.event})', ...
        eefig_adaptive.new_granules, ...
        'VariableNames', {'Run','Event','NewGranules'}));

    plotAdaptiveEefig(one_step.summary(frozen_idx,:), ...
        eefig_adaptive.one_step_summary, dynamics.summary, ...
        eefig_adaptive.summary, model_colors(frozen_idx,:), ...
        cfg.focus_horizon, cfg);
end

%% Selected propagation window

selected_window = struct();
if cfg.plot_selected_window
    selected_window = evaluateSelectedPropagationWindow(models,model_names, ...
        model_colors,runs,Ts,cfg,adaptive_color);
    plotSelectedPropagationWindow(selected_window);
end

eefig_reset();

fprintf('\nModel comparison complete. Results remain in the workspace as:\n');
fprintf(['  one_step, dynamics, end_to_end, eefig_adaptive, ' ...
    'selected_window, heading_quality\n']);

%%


%% Data and evaluation functions

function runs = buildRuns(datasets, Ts)
n_runs = numel(datasets);
runs = repmat(struct('path',"",'event',"",'laps',NaN, ...
    'track_id',NaN,'track_layout',"",'t',[],'psi',[],'vx',[], ...
    'vy',[],'r',[],'delta',[],'delta_dot',[],'st',[],'N',0), ...
    n_runs, 1);

for run_idx = 1:n_runs
    data = datasets(run_idx).data;
    required = {'time','psi','vx','vy','r','delta','st'};
    for field_idx = 1:numel(required)
        if ~isfield(data, required{field_idx})
            if strcmp(required{field_idx}, 'psi')
                error(['Evaluation data has no measured heading. Regenerate ' ...
                    'datasets with the current prepare_datasets.m.']);
            end
            error('Evaluation run %d is missing data.%s.', ...
                run_idx, required{field_idx});
        end
    end

    ini = datasets(run_idx).ini;
    fin = datasets(run_idx).fin;
    if isempty(ini), ini = 1; end
    if isempty(fin), fin = numel(data.vx); end
    fin = min(fin, numel(data.vx));
    if fin <= ini
        error('Evaluation run %d contains fewer than two selected samples.', ...
            run_idx);
    end
    idx = ini:fin;

    run.path = string(datasets(run_idx).path);
    run.event = string(datasets(run_idx).event);
    run.laps = double(datasets(run_idx).laps);
    run.track_id = double(datasets(run_idx).track_id);
    run.track_layout = string(datasets(run_idx).track_layout);
    run.t = data.time(idx(:));
    run.t = run.t(:) - run.t(1);
    run.psi = unwrap(data.psi(idx(:)));
    run.psi = run.psi(:) - run.psi(1);
    run.vx = data.vx(idx(:));
    run.vy = data.vy(idx(:));
    run.r = data.r(idx(:));
    run.delta = data.delta(idx(:));
    run.delta_dot = gradient(run.delta, Ts);
    run.st = data.st(idx(:));
    run.N = numel(idx);

    signals = [run.t,run.psi,run.vx,run.vy,run.r,run.delta, ...
        run.delta_dot,run.st];
    if any(~isfinite(signals), 'all')
        error('Evaluation run %d contains non-finite selected data.', run_idx);
    end
    runs(run_idx) = run;
    fprintf('Run %d: %s | %s | %g lap(s) | %d samples\n', ...
        run_idx,run.event,run.track_layout,run.laps,run.N);
end
end

function quality = headingQualityTable(runs, Ts)
n = numel(runs);
run_id = (1:n)';
event = string({runs.event})';
correlation = nan(n,1);
rmse_rate = nan(n,1);
bias_rate = nan(n,1);
p95_rate = nan(n,1);
for i = 1:n
    measured_increment = diff(runs(i).psi);
    expected_increment = 0.5*Ts* ...
        (runs(i).r(1:end-1)+runs(i).r(2:end));
    rate_error = (measured_increment-expected_increment)/Ts;
    heading_rate = measured_increment/Ts;
    r_mid = expected_increment/Ts;
    if std(heading_rate)>eps && std(r_mid)>eps
        correlation(i) = corr(heading_rate,r_mid);
    end
    rmse_rate(i) = sqrt(mean(rate_error.^2));
    bias_rate(i) = mean(rate_error);
    p95_rate(i) = prctile(abs(rate_error),95);
end
quality = table(run_id,event,correlation,rmse_rate,bias_rate,p95_rate, ...
    'VariableNames',{'Run','Event','Correlation','RMSE_rate', ...
    'Bias_rate','P95_rate'});
end

function result = evaluateOneStep(models, model_names, runs, Ts, suppress)
n_models = numel(model_names);
n_runs = numel(runs);
result.errors_vy = cell(n_models,n_runs);
result.errors_r = cell(n_models,n_runs);
result.errors_psi = cell(n_models,n_runs);
for run_idx = 1:n_runs
    run = runs(run_idx);
    n_pairs = run.N-1;
    for m = 1:n_models
        result.errors_vy{m,run_idx} = nan(n_pairs,1);
        result.errors_r{m,run_idx} = nan(n_pairs,1);
        result.errors_psi{m,run_idx} = nan(n_pairs,1);
    end
    for k = 1:n_pairs
        x = [0;run.vy(k);run.psi(k);run.r(k);run.delta(k);0];
        u = [run.vx(k),run.delta(k)];
        for m = 1:size(models,1)
            x_next = callModel(models{m,1},x,u,Ts,suppress);
            if all(isfinite(x_next([2,3,4])))
                result.errors_vy{m,run_idx}(k) = x_next(2)-run.vy(k+1);
                result.errors_r{m,run_idx}(k) = x_next(4)-run.r(k+1);
                result.errors_psi{m,run_idx}(k) = ...
                    wrapAngle(x_next(3)-run.psi(k+1));
            end
        end
        result.errors_vy{n_models,run_idx}(k) = run.vy(k)-run.vy(k+1);
        result.errors_r{n_models,run_idx}(k) = run.r(k)-run.r(k+1);
        result.errors_psi{n_models,run_idx}(k) = ...
            wrapAngle(run.psi(k)+Ts*run.r(k)-run.psi(k+1));
    end
end
result.vx = vertcatRuns(runs,'vx',true);
result.delta = vertcatRuns(runs,'delta',true);
result.vy = vertcatRuns(runs,'vy',true);
result.r = vertcatRuns(runs,'r',true);
end

function values = vertcatRuns(runs, field, remove_last)
values = [];
for i = 1:numel(runs)
    part = runs(i).(field);
    if remove_last, part = part(1:end-1); end
    values = [values;part(:)]; %#ok<AGROW>
end
end

function summary = summarizeOneStep(result, runs, model_names)
n_models = numel(model_names);
n_runs = numel(runs);
sample_rmse_vy = nan(n_models,1); sample_rmse_r = nan(n_models,1);
sample_mae_vy = nan(n_models,1); sample_mae_r = nan(n_models,1);
sample_p95_vy = nan(n_models,1); sample_p95_r = nan(n_models,1);
run_rmse_vy = nan(n_models,1); run_rmse_r = nan(n_models,1);
lap_rmse_vy = nan(n_models,1); lap_rmse_r = nan(n_models,1);
valid_fraction = nan(n_models,1);
per_run_vy = nan(n_models,n_runs); per_run_r = nan(n_models,n_runs);
weights = [runs.laps]';
for m = 1:n_models
    all_vy = vertcat(result.errors_vy{m,:});
    all_r = vertcat(result.errors_r{m,:});
    [sample_rmse_vy(m),sample_mae_vy(m),sample_p95_vy(m)] = ...
        errorMetrics(all_vy);
    [sample_rmse_r(m),sample_mae_r(m),sample_p95_r(m)] = ...
        errorMetrics(all_r);
    valid_fraction(m) = mean(isfinite(all_vy)&isfinite(all_r));
    for i = 1:n_runs
        per_run_vy(m,i) = rootMeanSquare(result.errors_vy{m,i});
        per_run_r(m,i) = rootMeanSquare(result.errors_r{m,i});
    end
    run_rmse_vy(m) = mean(per_run_vy(m,:),'omitnan');
    run_rmse_r(m) = mean(per_run_r(m,:),'omitnan');
    lap_rmse_vy(m) = weightedMean(per_run_vy(m,:)',weights);
    lap_rmse_r(m) = weightedMean(per_run_r(m,:)',weights);
end
skill_vy_percent = nan(n_models,1); skill_r_percent = nan(n_models,1);
baseline = find(model_names=="Persistence / constant yaw rate",1);
if ~isempty(baseline)
    baseline_vy = sample_rmse_vy(baseline);
    baseline_r = sample_rmse_r(baseline);
    skill_vy_percent = 100*(baseline_vy-sample_rmse_vy)/baseline_vy;
    skill_r_percent = 100*(baseline_r-sample_rmse_r)/baseline_r;
end
summary = table(model_names(:),sample_rmse_vy,sample_rmse_r, ...
    run_rmse_vy,run_rmse_r,lap_rmse_vy,lap_rmse_r,sample_mae_vy, ...
    sample_mae_r,sample_p95_vy,sample_p95_r,skill_vy_percent, ...
    skill_r_percent,valid_fraction,'VariableNames',{'Model', ...
    'SampleRMSE_vy','SampleRMSE_r','RunRMSE_vy','RunRMSE_r', ...
    'LapWeightedRMSE_vy','LapWeightedRMSE_r','MAE_vy','MAE_r', ...
    'P95_vy','P95_r','SkillVsPersistence_vy_percent', ...
    'SkillVsPersistence_r_percent','ValidFraction'});
summary.Properties.UserData.per_run_rmse_vy = per_run_vy;
summary.Properties.UserData.per_run_rmse_r = per_run_r;
end

function grouped = groupedOneStepTable(result, runs, model_names, field)
groups = unique(string({runs.(field)}),'stable');
rows = cell(numel(groups)*numel(model_names),5);
row = 0;
for g = 1:numel(groups)
    run_mask = string({runs.(field)})==groups(g);
    for m = 1:numel(model_names)
        row = row+1;
        evy = vertcat(result.errors_vy{m,run_mask});
        er = vertcat(result.errors_r{m,run_mask});
        rows(row,:) = {groups(g),model_names(m),rootMeanSquare(evy), ...
            rootMeanSquare(er),sum(isfinite(evy))};
    end
end
grouped = cell2table(rows,'VariableNames', ...
    {'Group','Model','RMSE_vy','RMSE_r','Samples'});
end

function result = evaluateRollingPropagation(models,model_names,runs,Ts,cfg,mode)
n_models = numel(model_names); n_runs = numel(runs);
n_horizons = numel(cfg.horizons); max_horizon = max(cfg.horizons);
horizon_slot = zeros(max_horizon,1);
horizon_slot(cfg.horizons) = 1:n_horizons;
result.mode = string(mode); result.horizons = cfg.horizons;
result.divergence_limits = [cfg.divergence_abs_vy,cfg.divergence_abs_r];
result.errors_vy = cell(n_models,n_horizons,n_runs);
result.errors_r = cell(n_models,n_horizons,n_runs);
result.errors_psi = cell(n_models,n_horizons,n_runs);
result.errors_delta = cell(n_models,n_horizons,n_runs);
result.start_indices = cell(n_runs,1);
for run_idx = 1:n_runs
    run = runs(run_idx);
    starts = (1:cfg.rolling_stride:(run.N-max_horizon))';
    result.start_indices{run_idx} = starts;
    n_starts = numel(starts);
    for m = 1:n_models
        for h = 1:n_horizons
            result.errors_vy{m,h,run_idx} = nan(n_starts,1);
            result.errors_r{m,h,run_idx} = nan(n_starts,1);
            result.errors_psi{m,h,run_idx} = nan(n_starts,1);
            result.errors_delta{m,h,run_idx} = nan(n_starts,1);
        end
    end
    for origin_idx = 1:n_starts
        origin = starts(origin_idx);
        for m = 1:size(models,1)
            if strcmp(mode,'measured_delta')
                x = [0;run.vy(origin);0;run.r(origin);run.delta(origin);0];
            else
                x = [0;run.vy(origin);0;run.r(origin);run.delta(origin); ...
                    run.delta_dot(origin)];
            end
            for step = 1:max_horizon
                k = origin+step-1;
                if strcmp(mode,'measured_delta')
                    x(5) = run.delta(k); x(6) = 0;
                    u = [run.vx(k),run.delta(k)];
                else
                    u = [run.vx(k),run.st(k)];
                end
                x = callModel(models{m,1},x,u,Ts,cfg.suppress_model_prints);
                if any(~isfinite(x)), break; end
                slot = horizon_slot(step);
                if slot>0
                    target = origin+step;
                    result.errors_vy{m,slot,run_idx}(origin_idx) = ...
                        x(2)-run.vy(target);
                    result.errors_r{m,slot,run_idx}(origin_idx) = ...
                        x(4)-run.r(target);
                    result.errors_psi{m,slot,run_idx}(origin_idx) = ...
                        wrapAngle(x(3)-(run.psi(target)-run.psi(origin)));
                    if strcmp(mode,'steering_command')
                        result.errors_delta{m,slot,run_idx}(origin_idx) = ...
                            x(5)-run.delta(target);
                    end
                end
            end
        end
        m = n_models;
        for slot = 1:n_horizons
            step = cfg.horizons(slot); target = origin+step;
            result.errors_vy{m,slot,run_idx}(origin_idx) = ...
                run.vy(origin)-run.vy(target);
            result.errors_r{m,slot,run_idx}(origin_idx) = ...
                run.r(origin)-run.r(target);
            result.errors_psi{m,slot,run_idx}(origin_idx) = ...
                wrapAngle(step*Ts*run.r(origin)- ...
                (run.psi(target)-run.psi(origin)));
            if strcmp(mode,'steering_command')
                result.errors_delta{m,slot,run_idx}(origin_idx) = ...
                    run.delta(origin)-run.delta(target);
            end
        end
    end
    fprintf('  %s run %d/%d: %d rolling origins\n', ...
        mode,run_idx,n_runs,n_starts);
end
end

function summary = summarizePropagation(result,runs,model_names,Ts)
n_models = numel(model_names); n_horizons = numel(result.horizons);
n_runs = numel(runs); n_rows = n_models*n_horizons;
model = strings(n_rows,1); horizon_steps = zeros(n_rows,1);
horizon_seconds = zeros(n_rows,1);
sample_rmse_vy = nan(n_rows,1); sample_rmse_r = nan(n_rows,1);
sample_rmse_psi = nan(n_rows,1); sample_rmse_delta = nan(n_rows,1);
run_rmse_vy = nan(n_rows,1); run_rmse_r = nan(n_rows,1);
run_rmse_psi = nan(n_rows,1);
lap_rmse_vy = nan(n_rows,1); lap_rmse_r = nan(n_rows,1);
lap_rmse_psi = nan(n_rows,1);
p95_vy = nan(n_rows,1); p95_r = nan(n_rows,1); p95_psi = nan(n_rows,1);
max_abs_vy = nan(n_rows,1); max_abs_r = nan(n_rows,1);
valid_fraction = nan(n_rows,1); bounded_fraction = nan(n_rows,1);
windows = zeros(n_rows,1);
weights = [runs.laps]'; row = 0;
for m = 1:n_models
    for h = 1:n_horizons
        row = row+1; model(row) = model_names(m);
        horizon_steps(row) = result.horizons(h);
        horizon_seconds(row) = result.horizons(h)*Ts;
        evy = vertcat(result.errors_vy{m,h,:});
        er = vertcat(result.errors_r{m,h,:});
        epsi = vertcat(result.errors_psi{m,h,:});
        edelta = vertcat(result.errors_delta{m,h,:});
        sample_rmse_vy(row) = rootMeanSquare(evy);
        sample_rmse_r(row) = rootMeanSquare(er);
        sample_rmse_psi(row) = rootMeanSquare(epsi);
        sample_rmse_delta(row) = rootMeanSquare(edelta);
        p95_vy(row) = percentileAbs(evy,95);
        p95_r(row) = percentileAbs(er,95);
        p95_psi(row) = percentileAbs(epsi,95);
        max_abs_vy(row) = maxAbsolute(evy);
        max_abs_r(row) = maxAbsolute(er);
        valid_fraction(row) = mean(isfinite(evy)&isfinite(er));
        bounded_fraction(row) = mean(isfinite(evy)&isfinite(er) & ...
            abs(evy)<=result.divergence_limits(1) & ...
            abs(er)<=result.divergence_limits(2));
        windows(row) = numel(evy);
        per_run_vy = nan(n_runs,1); per_run_r = nan(n_runs,1);
        per_run_psi = nan(n_runs,1);
        for i = 1:n_runs
            per_run_vy(i) = rootMeanSquare(result.errors_vy{m,h,i});
            per_run_r(i) = rootMeanSquare(result.errors_r{m,h,i});
            per_run_psi(i) = rootMeanSquare(result.errors_psi{m,h,i});
        end
        run_rmse_vy(row) = mean(per_run_vy,'omitnan');
        run_rmse_r(row) = mean(per_run_r,'omitnan');
        run_rmse_psi(row) = mean(per_run_psi,'omitnan');
        lap_rmse_vy(row) = weightedMean(per_run_vy,weights);
        lap_rmse_r(row) = weightedMean(per_run_r,weights);
        lap_rmse_psi(row) = weightedMean(per_run_psi,weights);
    end
end
skill_vy_percent = nan(n_rows,1); skill_r_percent = nan(n_rows,1);
baseline_name = "Persistence / constant yaw rate";
for h = 1:n_horizons
    baseline = model==baseline_name & horizon_steps==result.horizons(h);
    if any(baseline)
        rows = horizon_steps==result.horizons(h);
        skill_vy_percent(rows) = 100*(sample_rmse_vy(baseline) ...
            - sample_rmse_vy(rows))/sample_rmse_vy(baseline);
        skill_r_percent(rows) = 100*(sample_rmse_r(baseline) ...
            - sample_rmse_r(rows))/sample_rmse_r(baseline);
    end
end
summary = table(model,horizon_steps,horizon_seconds,sample_rmse_vy, ...
    sample_rmse_r,sample_rmse_psi,sample_rmse_delta,run_rmse_vy, ...
    run_rmse_r,run_rmse_psi,lap_rmse_vy,lap_rmse_r,lap_rmse_psi, ...
    p95_vy,p95_r,p95_psi,max_abs_vy,max_abs_r,valid_fraction, ...
    bounded_fraction,skill_vy_percent,skill_r_percent,windows, ...
    'VariableNames',{'Model','HorizonSteps','HorizonSeconds', ...
    'SampleRMSE_vy','SampleRMSE_r','SampleRMSE_psi','SampleRMSE_delta', ...
    'RunRMSE_vy','RunRMSE_r','RunRMSE_psi','LapWeightedRMSE_vy', ...
    'LapWeightedRMSE_r','LapWeightedRMSE_psi','P95_vy','P95_r', ...
    'P95_psi','MaxAbs_vy','MaxAbs_r','ValidFraction','BoundedFraction', ...
    'SkillVsPersistence_vy_percent','SkillVsPersistence_r_percent', ...
    'Windows'});
end

function table_out = focusHorizonTable(summary,horizon)
table_out = summary(summary.HorizonSteps==horizon,:);
end

function adaptive = attachPropagationBaselineSkill(adaptive,reference)
baseline = reference(reference.Model== ...
    "Persistence / constant yaw rate",:);
for row = 1:height(adaptive)
    match = baseline.HorizonSteps==adaptive.HorizonSteps(row);
    adaptive.SkillVsPersistence_vy_percent(row) = 100*( ...
        baseline.SampleRMSE_vy(match)-adaptive.SampleRMSE_vy(row)) ...
        / baseline.SampleRMSE_vy(match);
    adaptive.SkillVsPersistence_r_percent(row) = 100*( ...
        baseline.SampleRMSE_r(match)-adaptive.SampleRMSE_r(row)) ...
        / baseline.SampleRMSE_r(match);
end
end

function report = propagationDivergenceTable(summary)
names = unique(summary.Model,'stable');
model = strings(numel(names),1); first_horizon = nan(numel(names),1);
bounded_at_first = nan(numel(names),1);
bounded_at_max_horizon = nan(numel(names),1);
max_abs_vy = nan(numel(names),1); max_abs_r = nan(numel(names),1);
keep = false(numel(names),1);
for m = 1:numel(names)
    rows = find(summary.Model==names(m));
    bad = rows(summary.BoundedFraction(rows)<1);
    if isempty(bad), continue; end
    keep(m) = true; model(m) = names(m);
    [first_horizon(m),first_local] = min(summary.HorizonSteps(bad));
    first_row = bad(first_local);
    bounded_at_first(m) = summary.BoundedFraction(first_row);
    [~,last_local] = max(summary.HorizonSteps(rows));
    bounded_at_max_horizon(m) = summary.BoundedFraction(rows(last_local));
    max_abs_vy(m) = max(summary.MaxAbs_vy(rows),[],'omitnan');
    max_abs_r(m) = max(summary.MaxAbs_r(rows),[],'omitnan');
end
report = table(model(keep),first_horizon(keep),bounded_at_first(keep), ...
    bounded_at_max_horizon(keep),max_abs_vy(keep),max_abs_r(keep), ...
    'VariableNames',{'Model','FirstDivergentHorizon', ...
    'BoundedFractionAtFirst','BoundedFractionAtMaxHorizon', ...
    'MaximumAbsError_vy','MaximumAbsError_r'});
end

function diagnostic = ltvEulerStabilityDiagnostic(runs,Ts)
speeds = [0.2;1;2;3;5;10;20];
scheduled_speeds = max(speeds,3);
euler_radius = nan(size(speeds)); exact_radius = nan(size(speeds));
for i = 1:numel(speeds)
    [Ad,~,~] = ltv_matrix(zeros(6,1),0,speeds(i),Ts);
    Ad_lat = Ad([2,4],[2,4]);
    Ac_lat = (Ad_lat-eye(2))/Ts;
    euler_radius(i) = max(abs(eig(Ad_lat)));
    exact_radius(i) = max(abs(eig(expm(Ac_lat*Ts))));
end
grid_speed = linspace(0.1,30,3000)'; grid_radius = nan(size(grid_speed));
for i = 1:numel(grid_speed)
    [Ad,~,~] = ltv_matrix(zeros(6,1),0,grid_speed(i),Ts);
    grid_radius(i) = max(abs(eig(Ad([2,4],[2,4]))));
end
all_vx = vertcatRuns(runs,'vx',false);
diagnostic.spectral_radius = table(speeds,scheduled_speeds,euler_radius, ...
    exact_radius,'VariableNames',{'MeasuredLongitudinalSpeed', ...
    'ScheduledDynamicSpeed','EulerSpectralRadius','ExactSpectralRadius'});
diagnostic.maximum_euler_radius = max(grid_radius);
diagnostic.evaluation_fraction_clamped = mean(all_vx<3);
end

function grouped = groupedPropagationTable(result,runs,model_names,horizon,field)
[~,h] = ismember(horizon,result.horizons);
groups = unique(string({runs.(field)}),'stable');
rows = cell(numel(groups)*numel(model_names),7);
row = 0;
for g = 1:numel(groups)
    run_mask = string({runs.(field)})==groups(g);
    for m = 1:numel(model_names)
        row = row+1;
        evy = vertcat(result.errors_vy{m,h,run_mask});
        er = vertcat(result.errors_r{m,h,run_mask});
        epsi = vertcat(result.errors_psi{m,h,run_mask});
        rows(row,:) = {groups(g),model_names(m),horizon, ...
            rootMeanSquare(evy),rootMeanSquare(er), ...
            rootMeanSquare(epsi),sum(isfinite(evy))};
    end
end
grouped = cell2table(rows,'VariableNames', ...
    {'Group','Model','HorizonSteps','RMSE_vy','RMSE_r','RMSE_psi', ...
    'Windows'});
end

function adaptive = evaluateAdaptiveEefig(runs,Ts,cfg)
n_runs = numel(runs); n_horizons = numel(cfg.horizons);
max_horizon = max(cfg.horizons);
model_path = fullfile('models','eefig','eefig.mat');
adaptive.horizons = cfg.horizons; adaptive.mode = "measured_delta_adaptive";
adaptive.divergence_limits = [cfg.divergence_abs_vy,cfg.divergence_abs_r];
adaptive.errors_vy = cell(1,n_horizons,n_runs);
adaptive.errors_r = cell(1,n_horizons,n_runs);
adaptive.errors_psi = cell(1,n_horizons,n_runs);
adaptive.errors_delta = cell(1,n_horizons,n_runs);
adaptive.start_indices = cell(n_runs,1);
adaptive.new_granules = zeros(n_runs,1);
adaptive.one_step.errors_vy = cell(1,n_runs);
adaptive.one_step.errors_r = cell(1,n_runs);
adaptive.one_step.errors_psi = cell(1,n_runs);
for run_idx = 1:n_runs
    saved = load(model_path,'eefig_model');
    learner = saved.eefig_model.learner; learner.startNewRun();
    sx = saved.eefig_model.norm.scale_x(:);
    su = saved.eefig_model.norm.scale_u(:);
    run = runs(run_idx);
    starts = (1:cfg.rolling_stride:(run.N-max_horizon))';
    adaptive.start_indices{run_idx} = starts;
    origin_lookup = zeros(run.N,1); origin_lookup(starts) = 1:numel(starts);
    adaptive.one_step.errors_vy{1,run_idx} = nan(run.N-1,1);
    adaptive.one_step.errors_r{1,run_idx} = nan(run.N-1,1);
    adaptive.one_step.errors_psi{1,run_idx} = nan(run.N-1,1);
    for h = 1:n_horizons
        adaptive.errors_vy{1,h,run_idx} = nan(numel(starts),1);
        adaptive.errors_r{1,h,run_idx} = nan(numel(starts),1);
        adaptive.errors_psi{1,h,run_idx} = nan(numel(starts),1);
        adaptive.errors_delta{1,h,run_idx} = nan(numel(starts),1);
    end
    for k = 1:run.N-1
        origin_idx = origin_lookup(k);
        if origin_idx>0
            x_lat = [run.vy(k);run.r(k)]; psi_pred = 0;
            for step = 1:max_horizon
                q = k+step-1; u_lat = [run.vx(q);run.delta(q)];
                x_lat_next = learner.predict(x_lat./sx,u_lat./su).*sx;
                psi_pred = psi_pred+Ts*x_lat(2); x_lat = x_lat_next;
                slot = find(cfg.horizons==step,1);
                if ~isempty(slot)
                    target = k+step;
                    adaptive.errors_vy{1,slot,run_idx}(origin_idx) = ...
                        x_lat(1)-run.vy(target);
                    adaptive.errors_r{1,slot,run_idx}(origin_idx) = ...
                        x_lat(2)-run.r(target);
                    adaptive.errors_psi{1,slot,run_idx}(origin_idx) = ...
                        wrapAngle(psi_pred-(run.psi(target)-run.psi(k)));
                end
            end
        end
        x = [run.vy(k);run.r(k)]; u = [run.vx(k);run.delta(k)];
        target = [run.vy(k+1);run.r(k+1)];
        prediction = learner.predict(x./sx,u./su).*sx;
        adaptive.one_step.errors_vy{1,run_idx}(k) = prediction(1)-target(1);
        adaptive.one_step.errors_r{1,run_idx}(k) = prediction(2)-target(2);
        adaptive.one_step.errors_psi{1,run_idx}(k) = ...
            wrapAngle(run.psi(k)+Ts*prediction(2)-run.psi(k+1));
        status = learner.updateOnline(x./sx,u./su,target./sx);
        adaptive.new_granules(run_idx) = adaptive.new_granules(run_idx)+ ...
            status.created_new_granule;
    end
    fprintf('  adaptive EEFig run %d/%d: %d origins, %d new granules\n', ...
        run_idx,n_runs,numel(starts),adaptive.new_granules(run_idx));
end
end

function result = evaluateSelectedPropagationWindow(models,model_names, ...
        model_colors,runs,Ts,cfg,adaptive_color)
validateattributes(cfg.selected_run,{'numeric'}, ...
    {'scalar','integer','>=',1,'<=',numel(runs)});
validateattributes(cfg.selected_start,{'numeric'}, ...
    {'scalar','integer','>=',1});
validateattributes(cfg.selected_horizon,{'numeric'}, ...
    {'scalar','integer','positive'});

run = runs(cfg.selected_run);
origin = cfg.selected_start;
horizon = cfg.selected_horizon;
if origin+horizon>run.N
    error(['Selected propagation window exceeds run %d: start %d + ' ...
        'horizon %d, but the run contains %d samples.'], ...
        cfg.selected_run,origin,horizon,run.N);
end

include_adaptive = cfg.run_adaptive_eefig;
names = model_names(:);
colors = model_colors;
if include_adaptive
    names(end+1,1) = "EEFIG adaptive";
    colors(end+1,:) = adaptive_color;
end
n_series = numel(names);

result.names = names;
result.colors = colors;
result.run = cfg.selected_run;
result.start = origin;
result.horizon = horizon;
result.event = run.event;
result.track_layout = run.track_layout;
result.time = (0:horizon)'*Ts;
indices = origin:(origin+horizon);
result.measured_vy = run.vy(indices);
result.measured_r = run.r(indices);
result.measured_psi = run.psi(indices)-run.psi(origin);
result.measured_vx = run.vx(indices);
result.measured_delta = run.delta(indices);
result.predicted_vy = nan(horizon+1,n_series);
result.predicted_r = nan(horizon+1,n_series);
result.predicted_psi = nan(horizon+1,n_series);
result.predicted_vy(1,:) = run.vy(origin);
result.predicted_r(1,:) = run.r(origin);
result.predicted_psi(1,:) = 0;

for m = 1:size(models,1)
    x = [0;run.vy(origin);0;run.r(origin);run.delta(origin);0];
    for step = 1:horizon
        k = origin+step-1;
        x(5) = run.delta(k); x(6) = 0;
        x = callModel(models{m,1},x,[run.vx(k),run.delta(k)],Ts, ...
            cfg.suppress_model_prints);
        if any(~isfinite(x)), break; end
        result.predicted_vy(step+1,m) = x(2);
        result.predicted_r(step+1,m) = x(4);
        result.predicted_psi(step+1,m) = x(3);
    end
end

baseline = numel(model_names);
result.predicted_vy(:,baseline) = run.vy(origin);
result.predicted_r(:,baseline) = run.r(origin);
result.predicted_psi(:,baseline) = result.time*run.r(origin);

result.adaptive_prior_transitions = 0;
result.adaptive_new_granules_before_window = 0;
if include_adaptive
    saved = load(fullfile('models','eefig','eefig.mat'),'eefig_model');
    learner = saved.eefig_model.learner;
    learner.startNewRun();
    initial_granules = learner.NG;
    sx = saved.eefig_model.norm.scale_x(:);
    su = saved.eefig_model.norm.scale_u(:);

    % Adapt only from transitions strictly before the selected origin.
    for k = 1:(origin-1)
        x_measured = [run.vy(k);run.r(k)];
        u_measured = [run.vx(k);run.delta(k)];
        target = [run.vy(k+1);run.r(k+1)];
        learner.updateOnline(x_measured./sx,u_measured./su,target./sx);
    end
    result.adaptive_prior_transitions = origin-1;
    result.adaptive_new_granules_before_window = learner.NG-initial_granules;

    adaptive_idx = n_series;
    x_lat = [run.vy(origin);run.r(origin)];
    psi_relative = 0;
    for step = 1:horizon
        k = origin+step-1;
        u_lat = [run.vx(k);run.delta(k)];
        x_next = learner.predict(x_lat./sx,u_lat./su).*sx;
        psi_relative = psi_relative+Ts*x_lat(2);
        x_lat = x_next;
        result.predicted_vy(step+1,adaptive_idx) = x_lat(1);
        result.predicted_r(step+1,adaptive_idx) = x_lat(2);
        result.predicted_psi(step+1,adaptive_idx) = psi_relative;
    end
end

fprintf(['\nSelected propagation window: run %d (%s, %s), samples ' ...
    '%d-%d, %.2f s.\n'],cfg.selected_run,run.event,run.track_layout, ...
    origin,origin+horizon,horizon*Ts);
if include_adaptive
    fprintf(['Adaptive EEFig uses %d earlier transitions from this run; ' ...
        'its parameters are frozen inside the plotted window.\n'], ...
        result.adaptive_prior_transitions);
end
end

%% Plotting functions

function plotSelectedPropagationWindow(result)
figure('Name','Selected dynamics-only propagation window', ...
    'Position',[80 80 1550 900]);
layout = tiledlayout(3,2,'TileSpacing','compact','Padding','compact');

measured = {result.measured_vy,result.measured_r,result.measured_psi};
predicted = {result.predicted_vy,result.predicted_r, ...
    result.predicted_psi};
ylabels = {'v_y [m/s]','r [rad/s]','relative heading [rad]'};
titles = {'Lateral velocity propagation','Yaw-rate propagation', ...
    'Heading propagation'};
state_axes = gobjects(3,1);
for signal = 1:3
    state_axes(signal) = nexttile(layout);
    ax = state_axes(signal); hold(ax,'on'); grid(ax,'on');
    plot(ax,result.time,measured{signal},'k-','LineWidth',2.2, ...
        'DisplayName','Measured');
    for m = 1:numel(result.names)
        style = '-'; width = 1.15;
        if result.names(m)=="Persistence / constant yaw rate"
            style = ':'; width = 1.7;
        elseif result.names(m)=="EEFIG adaptive"
            style = '--'; width = 1.7;
        end
        plot(ax,result.time,predicted{signal}(:,m),style, ...
            'Color',result.colors(m,:),'LineWidth',width, ...
            'DisplayName',result.names(m));
    end
    xlabel(ax,'Time from window origin [s]');
    ylabel(ax,ylabels{signal}); title(ax,titles{signal});
end

ax = nexttile(layout); plot(ax,result.time,result.measured_delta, ...
    'k-','LineWidth',1.5); grid(ax,'on');
xlabel(ax,'Time from window origin [s]'); ylabel(ax,'\delta [rad]');
title(ax,'Measured steering supplied to every dynamics model');

ax = nexttile(layout); plot(ax,result.time,result.measured_vx, ...
    'k-','LineWidth',1.5); grid(ax,'on');
xlabel(ax,'Time from window origin [s]'); ylabel(ax,'v_x [m/s]');
title(ax,'Measured longitudinal speed supplied to every model');

ax = nexttile(layout); axis(ax,'off');
summary_text = sprintf([ ...
    'Run: %d\nEvent: %s\nTrack: %s\nSamples: %d--%d\n' ...
    'Horizon: %d steps (%.2f s)\n' ...
    'Adaptive prior transitions: %d\nNew adaptive granules before window: %d'], ...
    result.run,result.event,result.track_layout,result.start, ...
    result.start+result.horizon,result.horizon,result.time(end), ...
    result.adaptive_prior_transitions, ...
    result.adaptive_new_granules_before_window);
text(ax,0,0.95,summary_text,'Units','normalized', ...
    'VerticalAlignment','top','Interpreter','none','FontSize',10);

legend(state_axes(1),'Location','eastoutside');
title(layout,sprintf('Dynamics-only propagation: run %d, sample %d', ...
    result.run,result.start));
end

function plotOneStepSummary(summary,model_names,colors,cfg)
figure('Name','One-step model errors','Position',[100 100 1400 700]);
layout = tiledlayout(2,1,'TileSpacing','compact','Padding','compact');
plotMetricSummary(nexttile(layout),model_names,summary.SampleRMSE_r, ...
    summary.MAE_r,summary.P95_r,colors,'r error [rad/s]','Yaw rate',cfg);
plotMetricSummary(nexttile(layout),model_names,summary.SampleRMSE_vy, ...
    summary.MAE_vy,summary.P95_vy,colors,'v_y error [m/s]', ...
    'Lateral velocity',cfg);
end

function plotMetricSummary(ax,names,rmse,mae,p95,colors,ylab,ttl,cfg)
hold(ax,'on'); grid(ax,'on'); x = 1:numel(names);
bars = bar(ax,x,rmse,'FaceColor','flat','FaceAlpha',0.9, ...
    'DisplayName','RMSE'); bars.CData = colors;
for m = 1:numel(names)
    plot(ax,x(m),mae(m),'o','Color',colors(m,:), ...
        'MarkerFaceColor',colors(m,:),'MarkerEdgeColor','w', ...
        'MarkerSize',7,'HandleVisibility','off');
    plot(ax,x(m),p95(m),'x','Color',colors(m,:), ...
        'LineWidth',2,'MarkerSize',9,'HandleVisibility','off');
end
mae_key = plot(ax,nan,nan,'o','Color',[.8 .8 .8], ...
    'MarkerFaceColor',[.8 .8 .8],'DisplayName','MAE');
p95_key = plot(ax,nan,nan,'x','Color',[.8 .8 .8], ...
    'LineWidth',2,'DisplayName','P95(|e|)');
xticks(ax,x); xticklabels(ax,names); ylabel(ax,ylab); title(ax,ttl);
legend(ax,[bars,mae_key,p95_key],{'RMSE','MAE','P95(|e|)'}, ...
    'Location','best');
clampYAxisToNormalScale(ax,[rmse;mae;p95],cfg.plot_outlier_factor);
end

function plotOneStepResiduals(result,names,colors,cfg)
[vx_centers,vx_bin,vx_count] = makeBins(result.vx,cfg.vx_edges);
[delta_centers,delta_bin,delta_count] = ...
    makeBins(abs(result.delta),cfg.delta_edges);
figure('Name','One-step residuals by operating condition', ...
    'Position',[100 100 1450 850]);
layout = tiledlayout(3,2,'TileSpacing','compact','Padding','compact');
plotBinned(nexttile(layout),result.errors_vy,vx_bin,vx_centers, ...
    vx_count,cfg.min_bin_samples,names,colors,'v_x [m/s]', ...
    'RMSE e_{vy} [m/s]','v_y residual versus speed',cfg);
plotBinned(nexttile(layout),result.errors_r,vx_bin,vx_centers, ...
    vx_count,cfg.min_bin_samples,names,colors,'v_x [m/s]', ...
    'RMSE e_r [rad/s]','r residual versus speed',cfg);
plotBinned(nexttile(layout),result.errors_vy,delta_bin,delta_centers, ...
    delta_count,cfg.min_bin_samples,names,colors,'|\delta| [rad]', ...
    'RMSE e_{vy} [m/s]','v_y residual versus steering',cfg);
plotBinned(nexttile(layout),result.errors_r,delta_bin,delta_centers, ...
    delta_count,cfg.min_bin_samples,names,colors,'|\delta| [rad]', ...
    'RMSE e_r [rad/s]','r residual versus steering',cfg);
bar(nexttile(layout),vx_centers,vx_count); grid on;
xlabel('v_x [m/s]'); ylabel('Samples'); title('Speed-bin support');
bar(nexttile(layout),delta_centers,delta_count); grid on;
xlabel('|\delta| [rad]'); ylabel('Samples'); title('Steering-bin support');
end

function [centers,bins,count] = makeBins(values,edges)
values = values(:); edges = edges(:)';
if any(~isfinite(edges))
    cap = max([max(values,[],'omitnan'),max(edges(isfinite(edges)))])+eps;
    edges(~isfinite(edges)) = cap;
end
centers = 0.5*(edges(1:end-1)+edges(2:end));
bins = discretize(values,edges);
count = accumarray(bins(isfinite(bins)),1,[numel(centers),1])';
end

function plotBinned(ax,error_cells,bins,centers,count,min_count, ...
        names,colors,xlab,ylab,ttl,cfg)
hold(ax,'on'); grid(ax,'on');
all_values = nan(numel(names),numel(centers));
for m = 1:numel(names)
    error = vertcat(error_cells{m,:}); values = nan(size(centers));
    for b = 1:numel(centers)
        if count(b)>=min_count
            values(b) = rootMeanSquare(error(bins==b));
        end
    end
    all_values(m,:) = values;
    style = '-'; if m==numel(names), style = '--'; end
    plot(ax,centers,values,style,'Color',colors(m,:), ...
        'LineWidth',1.4,'DisplayName',names(m));
end
xlabel(ax,xlab); ylabel(ax,ylab); title(ax,ttl);
legend(ax,'Location','best');
clampYAxisToNormalScale(ax,all_values,cfg.plot_outlier_factor);
end

function plotPerRunOneStep(result,runs,names,colors,cfg)
figure('Name','One-step RMSE by held-out run','Position',[100 100 1450 700]);
layout = tiledlayout(2,1,'TileSpacing','compact','Padding','compact');
fields = {'errors_vy','errors_r'}; labels = {'v_y RMSE [m/s]','r RMSE [rad/s]'};
for tile = 1:2
    ax = nexttile(layout); hold(ax,'on'); grid(ax,'on');
    for m = 1:numel(names)
        values = nan(numel(runs),1);
        for i = 1:numel(runs)
            values(i) = rootMeanSquare(result.(fields{tile}){m,i});
        end
        style = '-o'; if m==numel(names), style = '--o'; end
        plot(ax,1:numel(runs),values,style,'Color',colors(m,:), ...
            'LineWidth',1.2,'DisplayName',names(m));
    end
    xticks(ax,1:numel(runs)); ylabel(ax,labels{tile});
    title(ax,'Each run has equal visual weight');
    clampYAxisToNormalScale(ax,valuesForRunPlot(result,fields{tile},runs), ...
        cfg.plot_outlier_factor);
end
xlabel(nexttile(layout,2),'Evaluation run');
legend(nexttile(layout,1),'Location','eastoutside');
end

function values = valuesForRunPlot(result,field,runs)
values = nan(size(result.(field),1),numel(runs));
for m = 1:size(values,1)
    for i = 1:numel(runs)
        values(m,i) = rootMeanSquare(result.(field){m,i});
    end
end
end

function plotPropagationCurves(summary,names,colors,focus_horizon,figure_title,cfg)
figure('Name',figure_title,'Position',[100 100 1450 900]);
layout = tiledlayout(3,2,'TileSpacing','compact','Padding','compact');
metrics = {'SampleRMSE_vy','SampleRMSE_r','SampleRMSE_psi', ...
    'RunRMSE_vy','RunRMSE_r','RunRMSE_psi'};
titles = {'Sample-weighted v_y','Sample-weighted r', ...
    'Sample-weighted heading','Run-weighted v_y','Run-weighted r', ...
    'Run-weighted heading'};
ylabs = {'RMSE [m/s]','RMSE [rad/s]','RMSE [rad]', ...
    'RMSE [m/s]','RMSE [rad/s]','RMSE [rad]'};
for tile = 1:6
    ax = nexttile(layout); hold(ax,'on'); grid(ax,'on');
    for m = 1:numel(names)
        rows = summary.Model==names(m);
        style = '-o'; if m==numel(names), style = '--o'; end
        plot(ax,summary.HorizonSeconds(rows),summary.(metrics{tile})(rows), ...
            style,'Color',colors(m,:),'LineWidth',1.4, ...
            'DisplayName',names(m));
    end
    xline(ax,focus_horizon*summary.HorizonSeconds(1)/ ...
        summary.HorizonSteps(1),':','MPC horizon');
    xlabel(ax,'Prediction horizon [s]'); ylabel(ax,ylabs{tile});
    title(ax,titles{tile});
    clampYAxisToNormalScale(ax,summary.(metrics{tile}), ...
        cfg.plot_outlier_factor);
end
legend(nexttile(layout,1),'Location','eastoutside');
title(layout,figure_title);
end

function plotSteeringActuatorDiagnostic(summary,names,colors,focus_horizon,cfg)
figure('Name','Steering-actuator propagation error', ...
    'Position',[100 100 1200 550]);
ax = axes; hold(ax,'on'); grid(ax,'on');
for m = 1:numel(names)
    rows = summary.Model==names(m);
    style = '-o'; if m==numel(names), style = '--o'; end
    plot(ax,summary.HorizonSeconds(rows), ...
        summary.SampleRMSE_delta(rows),style,'Color',colors(m,:), ...
        'LineWidth',1.4,'DisplayName',names(m));
end
Ts_plot = summary.HorizonSeconds(1)/summary.HorizonSteps(1);
xline(ax,focus_horizon*Ts_plot,':','MPC horizon');
xlabel(ax,'Prediction horizon [s]');
ylabel(ax,'Measured steering-position RMSE [rad]');
title(ax,'Separate steering-command/actuator diagnostic');
legend(ax,'Location','eastoutside');
clampYAxisToNormalScale(ax,summary.SampleRMSE_delta, ...
    cfg.plot_outlier_factor);
end

function plotFocusAggregation(focus,names,colors,figure_title,cfg)
figure('Name',[figure_title ' aggregation'],'Position',[100 100 1400 850]);
layout = tiledlayout(3,1,'TileSpacing','compact','Padding','compact');
plotAggregationTile(nexttile(layout),focus,names,colors,'vy', ...
    'v_y RMSE [m/s]',cfg);
plotAggregationTile(nexttile(layout),focus,names,colors,'r', ...
    'r RMSE [rad/s]',cfg);
plotHeadingAggregationTile(nexttile(layout),focus,names,colors,cfg);
title(layout,figure_title);
end

function plotAggregationTile(ax,focus,names,colors,suffix,ylab,cfg)
hold(ax,'on'); grid(ax,'on'); x = 1:numel(names);
sample = focus.(['SampleRMSE_' suffix]);
run = focus.(['RunRMSE_' suffix]);
lap = focus.(['LapWeightedRMSE_' suffix]);
bars = bar(ax,x,sample,'FaceColor','flat','DisplayName','Sample-weighted');
bars.CData = colors;
plot(ax,x,run,'ko','MarkerFaceColor','w','DisplayName','Run-weighted');
plot(ax,x,lap,'kd','MarkerFaceColor',[.8 .8 .8], ...
    'DisplayName','Declared-lap weighted');
xticks(ax,x); xticklabels(ax,names); ylabel(ax,ylab);
legend(ax,'Location','best');
clampYAxisToNormalScale(ax,[sample;run;lap],cfg.plot_outlier_factor);
end

function plotHeadingAggregationTile(ax,focus,names,colors,cfg)
hold(ax,'on'); grid(ax,'on'); x = 1:numel(names);
bars = bar(ax,x,focus.SampleRMSE_psi,'FaceColor','flat', ...
    'DisplayName','Sample-weighted');
bars.CData = colors;
plot(ax,x,focus.RunRMSE_psi,'ko','MarkerFaceColor','w', ...
    'DisplayName','Run-weighted');
plot(ax,x,focus.LapWeightedRMSE_psi,'kd', ...
    'MarkerFaceColor',[.8 .8 .8], ...
    'DisplayName','Declared-lap weighted');
xticks(ax,x); xticklabels(ax,names); ylabel(ax,'heading RMSE [rad]');
legend(ax,'Location','best');
clampYAxisToNormalScale(ax,[focus.SampleRMSE_psi;focus.RunRMSE_psi; ...
    focus.LapWeightedRMSE_psi],cfg.plot_outlier_factor);
end

function plotAdaptiveEefig(frozen_one,adaptive_one,frozen_prop, ...
        adaptive_prop,color,focus_horizon,cfg)
figure('Name','Frozen versus adaptive EEFig','Position',[100 100 1200 750]);
layout = tiledlayout(2,2,'TileSpacing','compact','Padding','compact');

% Rows are predicted states; columns are evaluation types:
%   [v_y one-step] [v_y propagation]
%   [r   one-step] [r   propagation]
ax = nexttile(layout); bar(ax,[frozen_one.SampleRMSE_vy, ...
    adaptive_one.SampleRMSE_vy],'FaceColor',color); grid(ax,'on');
xticklabels(ax,{'Frozen','Adaptive'}); ylabel(ax,'v_y RMSE [m/s]');
title(ax,'v_y: causal one-step');
clampYAxisToNormalScale(ax,[frozen_one.SampleRMSE_vy; ...
    adaptive_one.SampleRMSE_vy],cfg.plot_outlier_factor);

plotAdaptiveCurve(nexttile(layout),frozen_prop,adaptive_prop,color, ...
    'SampleRMSE_vy','v_y RMSE [m/s]',focus_horizon,cfg, ...
    'v_y: error versus horizon');

ax = nexttile(layout); bar(ax,[frozen_one.SampleRMSE_r, ...
    adaptive_one.SampleRMSE_r],'FaceColor',color); grid(ax,'on');
xticklabels(ax,{'Frozen','Adaptive'}); ylabel(ax,'r RMSE [rad/s]');
title(ax,'r: causal one-step');
clampYAxisToNormalScale(ax,[frozen_one.SampleRMSE_r; ...
    adaptive_one.SampleRMSE_r],cfg.plot_outlier_factor);

plotAdaptiveCurve(nexttile(layout),frozen_prop,adaptive_prop,color, ...
    'SampleRMSE_r','r RMSE [rad/s]',focus_horizon,cfg, ...
    'r: error versus horizon');
title(layout,'Frozen versus causally adaptive EEFig');
end

function plotAdaptiveCurve(ax,frozen,adaptive,color,metric,ylab, ...
        focus_horizon,cfg,plot_title)
frozen = frozen(frozen.Model=="EEFIG offline",:);
hold(ax,'on'); grid(ax,'on');
plot(ax,frozen.HorizonSeconds,frozen.(metric),'-o','Color',color, ...
    'LineWidth',1.5,'DisplayName','Frozen');
plot(ax,adaptive.HorizonSeconds,adaptive.(metric),'--o','Color',color, ...
    'LineWidth',1.5,'DisplayName','Causally adaptive');
Ts_plot = frozen.HorizonSeconds(1)/frozen.HorizonSteps(1);
xline(ax,focus_horizon*Ts_plot,':','MPC horizon', ...
    'HandleVisibility','off');
xlabel(ax,'Prediction horizon [s]'); ylabel(ax,ylab);
title(ax,plot_title);
legend(ax,'Location','best');
clampYAxisToNormalScale(ax,[frozen.(metric);adaptive.(metric)], ...
    cfg.plot_outlier_factor);
end

%% Small numerical helpers

function clampYAxisToNormalScale(ax,values,outlier_factor)
values = values(:);
finite_values = values(isfinite(values) & values>=0);
if isempty(finite_values), return; end
typical = median(finite_values);
threshold = outlier_factor*max(typical,eps);
normal_values = finite_values(finite_values<=threshold);
if isempty(normal_values)
    normal_values = min(finite_values);
end
is_off_scale = isinf(values) | (isfinite(values) & values>threshold);
if ~any(is_off_scale), return; end
upper = 1.10*max(normal_values);
if ~(isfinite(upper) && upper>0), return; end
ylim(ax,[0,upper]);
text(ax,0.99,0.97,sprintf('%d off-scale value(s); see tables', ...
    sum(is_off_scale)),'Units','normalized','HorizontalAlignment','right', ...
    'VerticalAlignment','top','FontSize',8,'Color',[0.8,0.2,0.2], ...
    'BackgroundColor','none');
end

function x_next = callModel(model_fcn,x,u,Ts,suppress)
if suppress
    [~,x_next] = evalc('model_fcn(x,u,Ts)');
else
    x_next = model_fcn(x,u,Ts);
end
x_next = x_next(:);
end

function [rmse,mae,p95] = errorMetrics(error)
error = error(isfinite(error));
if isempty(error), rmse=NaN; mae=NaN; p95=NaN; return; end
rmse = sqrt(mean(error.^2)); mae = mean(abs(error));
p95 = prctile(abs(error),95);
end

function value = rootMeanSquare(error)
error = error(isfinite(error));
if isempty(error), value=NaN; else, value=sqrt(mean(error.^2)); end
end

function value = percentileAbs(error,p)
error = error(isfinite(error));
if isempty(error), value=NaN; else, value=prctile(abs(error),p); end
end

function value = maxAbsolute(error)
if any(isinf(error)), value=Inf; return; end
error = error(isfinite(error));
if isempty(error), value=NaN; else, value=max(abs(error)); end
end

function value = weightedMean(values,weights)
valid = isfinite(values)&isfinite(weights)&weights>0;
if ~any(valid), value=NaN; return; end
value = sum(values(valid).*weights(valid))/sum(weights(valid));
end

function angle = wrapAngle(angle)
angle = atan2(sin(angle),cos(angle));
end
