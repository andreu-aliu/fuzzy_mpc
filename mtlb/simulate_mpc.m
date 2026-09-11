clear; clc;
mtlb_dir = '/home/andreu/ros_ws/src/as/control/fuzzy_mpc/mtlb';
cd(mtlb_dir);
addpath(genpath(mtlb_dir));

%% SETUP
dataset_file = fullfile(mtlb_dir,'data','datasets_evaluation.mat');
results_database_file = fullfile(mtlb_dir,'mpc_results.mat');
run_selection = [1 2]; % "all", one index, or a vector such as [1 3 5]

% Automatic per-run interval.
use_full_run = true;
trim_start_seconds = 2.0;
trim_end_seconds = 0.5;
max_mpc_windows_per_run = 200; % Inf evaluates the complete run

% Used only when use_full_run is false.
idx_start = 3000;
window_sample_count = 500;

% Run indices displayed by the two PLOT sections. This does not affect
% simulation or database storage.
plot_run_indices = 1;

Np = 60;
nx = 6;
nu = 1;
dt = 0.02;
ds = 0.025;

cfg = struct( ...
    'use_full_run',use_full_run, ...
    'trim_start_seconds',trim_start_seconds, ...
    'trim_end_seconds',trim_end_seconds, ...
    'max_mpc_windows_per_run',max_mpc_windows_per_run, ...
    'idx_start',idx_start, ...
    'window_sample_count',window_sample_count, ...
    'Np',Np,'nx',nx,'nu',nu,'dt',dt,'path_spacing',ds, ...
    'is_batch',false);

validateattributes(max_mpc_windows_per_run,{'numeric'}, ...
    {'scalar','real','positive'},mfilename,'max_mpc_windows_per_run');
assert(isinf(max_mpc_windows_per_run) || ...
    max_mpc_windows_per_run==fix(max_mpc_windows_per_run), ...
    'max_mpc_windows_per_run must be a positive integer or Inf.');

debug_opts.enabled = false;
debug_opts.step = [];
debug_opts.pause = false;
debug_opts.figure_id = 99;

comp_opts.enabled = false;
comp_opts.step = [];
comp_opts.pause = false;
comp_opts.figure_id = 98;

%% LOAD DATA

source = load(dataset_file,'datasets','meta');
assert(isfield(source,'datasets') && isfield(source,'meta'), ...
    '%s must contain datasets and meta.',dataset_file);
assert(abs(source.meta.Ts-dt)<eps(max(source.meta.Ts,dt)), ...
    'Dataset sample time %.6g s does not match MPC Ts %.6g s.', ...
    source.meta.Ts,dt);

if isstring(run_selection) || ischar(run_selection)
    assert(strcmpi(string(run_selection),"all"), ...
        'Text run_selection must be "all". Otherwise use numeric indices.');
    evaluation_runs = 1:numel(source.datasets);
else
    evaluation_runs = double(run_selection(:).');
    assert(~isempty(evaluation_runs) && all(isfinite(evaluation_runs)) && ...
        all(evaluation_runs==fix(evaluation_runs)) && ...
        all(evaluation_runs>=1) && ...
        all(evaluation_runs<=numel(source.datasets)), ...
        'run_selection contains an invalid evaluation-run index.');
    assert(numel(unique(evaluation_runs))==numel(evaluation_runs), ...
        'run_selection contains duplicate run indices.');
end
cfg.is_batch = numel(evaluation_runs)>1;
fprintf('Loaded %d held-out run(s) for MPC evaluation.\n', ...
    numel(evaluation_runs));

%% MPC PARAMETERS AND OPTIONS

params.n_horizon = Np;
params.Ts = dt;
params.model = "eefig"; % ltv anfis_direct anfis_delta anfis_derivative anfis_ltv_residual eefig
params.eefig_adaptive = false;
% Plant options: nonlinear_bicycle_linear_tire, nonlinear_bicycle,
% anfis_delta, ltv.
plant_model = "nonlinear_bicycle_linear_tire";

params.scale_y = 0.1;
params.scale_vy = 0.1;
params.scale_psi = 0.05;
params.scale_r = 0.05;
params.scale_st = 0.2;
params.scale_dst = 0.001;

params.q_y = 200;
params.q_vy = 0;
params.q_psi = 0;
params.q_r = 1;
params.q_st = 0;
params.q_dst = 0;

params.p_y = 1000;
params.p_vy = 0;
params.p_psi = 0;
params.p_r = 0;
params.p_st = 0;
params.p_dst = 0;

params.r_st = 1;
params.rd_st = 2;

params.min_st = -0.38;
params.max_st = 0.38;
params.max_delta = 0.45;
params.max_st_rate = 1.396;

%% SIMULATE

assert(exist('source','var')==1 && exist('evaluation_runs','var')==1, ...
    'Run LOAD DATA before SIMULATE.');
assert(exist('params','var')==1 && exist('plant_model','var')==1, ...
    'Run MPC PARAMETERS AND OPTIONS before SIMULATE.');

scenario_results = cell(numel(evaluation_runs),1);
for scenario_idx = 1:numel(evaluation_runs)
    evaluation_run = evaluation_runs(scenario_idx);
    scenario_results{scenario_idx} = run_mpc_scenario( ...
        source.datasets(evaluation_run),evaluation_run,dataset_file,cfg, ...
        params,plant_model,debug_opts,comp_opts);
end
fprintf('Completed %d MPC scenario(s).\n',numel(scenario_results));

%% PLOT INPUT DATA

assert(exist('scenario_results','var')==1 && ~isempty(scenario_results), ...
    'Run SIMULATE before PLOT INPUT DATA.');
results_to_plot = select_results(scenario_results,plot_run_indices);
for plot_idx = 1:numel(results_to_plot)
    plot_scenario_input(results_to_plot{plot_idx});
end

%% PLOT RESULTS

assert(exist('scenario_results','var')==1 && ~isempty(scenario_results), ...
    'Run SIMULATE before PLOT RESULTS.');
results_to_plot = select_results(scenario_results,plot_run_indices);
for plot_idx = 1:numel(results_to_plot)
    plot_scenario_result(results_to_plot{plot_idx});
end

%% PERFORMANCE INSIGHTS

assert(exist('scenario_results','var')==1 && ~isempty(scenario_results), ...
    'Run SIMULATE before PERFORMANCE INSIGHTS.');

result_entries = table();
for scenario_idx = 1:numel(scenario_results)
    result = scenario_results{scenario_idx};
    setup = result.setup;
    insights = result.insights;
    selected_run = result.selected_run;

    controller_setup = rmfield(setup, ...
        {'EvaluationRun','WindowStart','SampleCount'});
    scenario_setup = struct( ...
        'DatasetFile',setup.DatasetFile, ...
        'EvaluationRun',setup.EvaluationRun, ...
        'WindowStart',setup.WindowStart, ...
        'SampleCount',setup.SampleCount, ...
        'Event',char(selected_run.event), ...
        'TrackLayout',char(selected_run.track_layout), ...
        'DeclaredLaps',double(selected_run.laps));
    controller_key = string(jsonencode(controller_setup));
    scenario_key = string(jsonencode(scenario_setup));
    setup_key = string(jsonencode(setup));

    entry = addvars(insights,setup_key,controller_key,scenario_key, ...
        datetime('now'),1,string(setup.DatasetFile), ...
        string(setup.EvaluationProtocol),numel(source.datasets), ...
        setup.WindowStart,setup.SampleCount,setup.HorizonSteps,setup.Ts, ...
        string(setup.PlantModel), ...
        'Before','Model','NewVariableNames',{ ...
        'SetupKey','ControllerKey','ScenarioKey','UpdatedAt','UpdateCount', ...
        'DatasetFile','EvaluationProtocol','DatasetScenarioCount', ...
        'WindowStart','SampleCount','HorizonSteps','Ts_s','PlantModel'});
    if isempty(result_entries)
        result_entries = entry;
    else
        result_entries = [result_entries;entry]; %#ok<AGROW>
    end
end

fprintf('\nMPC PERFORMANCE INSIGHTS\n');
first_metric = find(strcmp(result_entries.Properties.VariableNames,'Model'));
disp(result_entries(:,first_metric:width(result_entries)));

database_file = results_database_file;
if isfile(database_file)
    stored_results = load(database_file,'mpc_results_database');
    assert(isfield(stored_results,'mpc_results_database'), ...
        '%s does not contain mpc_results_database.',database_file);
    mpc_results_database = upgrade_results_database( ...
        stored_results.mpc_results_database,result_entries(1,:));
else
    mpc_results_database = result_entries([],:);
end

created_count = 0;
updated_count = 0;
for entry_idx = 1:height(result_entries)
    matching_entry = mpc_results_database.SetupKey== ...
        result_entries.SetupKey(entry_idx);
    assert(nnz(matching_entry)<=1, ...
        'The MPC results database contains a duplicate setup key.');
    if any(matching_entry)
        result_entries.UpdateCount(entry_idx) = ...
            mpc_results_database.UpdateCount(matching_entry)+1;
        mpc_results_database(matching_entry,:) = result_entries(entry_idx,:);
        updated_count = updated_count+1;
    else
        mpc_results_database = [mpc_results_database; ...
            result_entries(entry_idx,:)]; %#ok<AGROW>
        created_count = created_count+1;
    end
end

mpc_results_summary = build_results_summary(mpc_results_database);
save(database_file,'mpc_results_database','mpc_results_summary');
fprintf('Database: %d created, %d updated, %d total scenario rows.\n', ...
    created_count,updated_count,height(mpc_results_database));

current_controller_keys = unique(result_entries.ControllerKey,'stable');
fprintf('\nALL-SCENARIO SUMMARY FOR CURRENT CONTROLLER SETUP(S)\n');
disp(mpc_results_summary(ismember( ...
    mpc_results_summary.ControllerKey,current_controller_keys),:));
%%

%% CLOSED-LOOP MODEL COMPARISON

% This section is database-only and can be run without executing SETUP,
% LOAD DATA, or SIMULATE. If the database contains several experiments
% (different horizons, weights, plants, window caps, etc.), [] selects the
% most recently updated experiment containing at least two model variants.
comparison_group_id = [];
comparison_only_complete = false;

if ~exist('mtlb_dir','var')
    mtlb_dir = '/home/andreu/ros_ws/src/as/control/fuzzy_mpc/mtlb';
end
if ~exist('results_database_file','var')
    results_database_file = fullfile(mtlb_dir,'mpc_results.mat');
end
comparison_database_file = results_database_file;
assert(isfile(comparison_database_file), ...
    'No MPC results database exists at %s.',comparison_database_file);
comparison_store = load(comparison_database_file, ...
    'mpc_results_database','mpc_results_summary');
assert(isfield(comparison_store,'mpc_results_database'), ...
    'The database does not contain mpc_results_database.');
comparison_database = comparison_store.mpc_results_database;
required_identity = {'ControllerKey','EvaluationProtocol'};
assert(all(ismember(required_identity, ...
    comparison_database.Properties.VariableNames)), ...
    ['The database uses the legacy schema. Run PERFORMANCE INSIGHTS once ' ...
    'after a simulation to upgrade it.']);
if isfield(comparison_store,'mpc_results_summary')
    comparison_summary_all = comparison_store.mpc_results_summary;
else
    comparison_summary_all = build_results_summary(comparison_database);
end
assert(~isempty(comparison_summary_all), ...
    'The MPC results database contains no controller results.');

% A comparison group contains configurations that differ only in prediction
% model and EEFig adaptation mode. Everything else must be identical.
comparison_keys = strings(height(comparison_summary_all),1);
for row = 1:height(comparison_summary_all)
    comparable_setup = jsondecode( ...
        comparison_summary_all.ControllerKey(row));
    changing_fields = intersect({'PredictionModel','EEFigAdaptive'}, ...
        fieldnames(comparable_setup),'stable');
    comparable_setup = rmfield(comparable_setup,changing_fields);
    comparison_keys(row) = string(jsonencode(comparable_setup));
end
[unique_comparison_keys,~,comparison_group_numbers] = ...
    unique(comparison_keys,'stable');

n_groups = numel(unique_comparison_keys);
group_id = (1:n_groups).';
group_protocol = strings(n_groups,1);
group_plant = strings(n_groups,1);
group_horizon = NaN(n_groups,1);
group_models = zeros(n_groups,1);
group_complete_models = zeros(n_groups,1);
group_last_update = NaT(n_groups,1);
for group_idx = 1:n_groups
    summary_rows = comparison_group_numbers==group_idx;
    group_protocol(group_idx) = ...
        comparison_summary_all.EvaluationProtocol(find(summary_rows,1));
    group_plant(group_idx) = ...
        comparison_summary_all.PlantModel(find(summary_rows,1));
    group_horizon(group_idx) = ...
        comparison_summary_all.HorizonSteps(find(summary_rows,1));
    group_models(group_idx) = nnz(summary_rows);
    group_complete_models(group_idx) = nnz(summary_rows & ...
        comparison_summary_all.ScenarioCoveragePercent>=100-1e-9);
    controller_keys_in_group = ...
        comparison_summary_all.ControllerKey(summary_rows);
    database_rows = ismember(comparison_database.ControllerKey, ...
        controller_keys_in_group);
    group_last_update(group_idx) = ...
        max(comparison_database.UpdatedAt(database_rows));
end
comparison_groups = table(group_id,group_protocol,group_plant, ...
    group_horizon,group_models,group_complete_models,group_last_update, ...
    'VariableNames',{'GroupID','Protocol','Plant','HorizonSteps', ...
    'ModelVariants','CompleteVariants','LastUpdate'});
fprintf('\nAVAILABLE CLOSED-LOOP COMPARISON GROUPS\n');
disp(comparison_groups);

if isempty(comparison_group_id)
    candidates = find(group_models>=2);
    if isempty(candidates)
        candidates = (1:n_groups).';
    end
    [~,latest_idx] = max(group_last_update(candidates));
    selected_group = candidates(latest_idx);
else
    validateattributes(comparison_group_id,{'numeric'}, ...
        {'scalar','integer','>=',1,'<=',n_groups},mfilename, ...
        'comparison_group_id');
    selected_group = comparison_group_id;
end

comparison_rows = comparison_group_numbers==selected_group;
comparison_summary = comparison_summary_all(comparison_rows,:);
if comparison_only_complete
    incomplete = ~isfinite(comparison_summary.ScenarioCoveragePercent) | ...
        comparison_summary.ScenarioCoveragePercent<100-1e-9;
    if any(incomplete)
        warning('Excluding incomplete model variants: %s',strjoin( ...
            cellstr(comparison_summary.Model(incomplete)),', '));
        comparison_summary = comparison_summary(~incomplete,:);
    end
end
assert(~isempty(comparison_summary), ...
    'Selected group has no model variants satisfying the coverage filter.');

% Supplement the controller-level summary with tail and per-run metrics
% retained in the scenario database.
n_models = height(comparison_summary);
mean_run_p95 = NaN(n_models,1);
worst_abs_error = NaN(n_models,1);
mean_mpc_p95 = NaN(n_models,1);
for model_idx = 1:n_models
    model_rows = comparison_database.ControllerKey== ...
        comparison_summary.ControllerKey(model_idx);
    model_scenarios = comparison_database(model_rows,:);
    mean_run_p95(model_idx) = ...
        mean(model_scenarios.LateralP95_m,'omitnan');
    worst_abs_error(model_idx) = ...
        max(model_scenarios.LateralMaxAbs_m,[],'omitnan');
    mean_mpc_p95(model_idx) = ...
        mean(model_scenarios.MPCTimeP95_ms,'omitnan');
end
comparison_table = addvars(comparison_summary,mean_run_p95, ...
    worst_abs_error,mean_mpc_p95, ...
    'After','SampleWeightedLateralRMSE_m', ...
    'NewVariableNames',{'MeanRunLateralP95_m', ...
    'WorstAbsoluteLateralError_m','MeanRunMPCTimeP95_ms'});
comparison_table = sortrows(comparison_table, ...
    'RunBalancedLateralRMSE_m','ascend');

fprintf('\nSELECTED CLOSED-LOOP MODEL COMPARISON: GROUP %d\n',selected_group);
display_columns = {'Model','ScenarioCount','ExpectedScenarioCount', ...
    'ScenarioCoveragePercent','RunBalancedLateralRMSE_m', ...
    'SampleWeightedLateralRMSE_m','MedianRunLateralRMSE_m', ...
    'WorstRunLateralRMSE_m','MeanRunLateralP95_m', ...
    'WorstAbsoluteLateralError_m','RunBalancedMeanModelError', ...
    'RunBalancedMeanMPCTime_ms','MeanRunMPCTimeP95_ms', ...
    'WorstOnlineCycleTime_ms','RunBalancedDeadlineMissPercent', ...
    'TotalSolverFailures'};
disp(comparison_table(:,display_columns));

model_labels = comparison_table.Model;
model_colors = closed_loop_model_colors(model_labels);
x = (1:n_models).';

figure('Name','Closed-loop aggregate model comparison', ...
    'Position',[100 100 1500 850]);
aggregate_layout = tiledlayout(2,2, ...
    'TileSpacing','compact','Padding','compact');

ax1 = nexttile(aggregate_layout); hold(ax1,'on'); grid(ax1,'on');
b1 = bar(ax1,x,comparison_table.RunBalancedLateralRMSE_m,0.65, ...
    'FaceColor','flat','DisplayName','Equal-run mean RMSE');
b1.CData = model_colors;
plot(ax1,x,comparison_table.SampleWeightedLateralRMSE_m,'ko', ...
    'MarkerFaceColor','w','DisplayName','Sample-weighted RMSE');
plot(ax1,x,comparison_table.MedianRunLateralRMSE_m,'k^', ...
    'MarkerFaceColor',[0.8 0.8 0.8],'DisplayName','Median run RMSE');
plot(ax1,x,comparison_table.WorstRunLateralRMSE_m,'kx', ...
    'LineWidth',1.8,'MarkerSize',8,'DisplayName','Worst run RMSE');
ylabel(ax1,'Lateral error [m]'); title(ax1,'Closed-loop tracking accuracy');
legend(ax1,'Location','best'); set_model_ticks(ax1,model_labels);

ax2 = nexttile(aggregate_layout); hold(ax2,'on'); grid(ax2,'on');
b2 = bar(ax2,x,comparison_table.MeanRunLateralP95_m,0.65, ...
    'FaceColor','flat','DisplayName','Mean run P95');
b2.CData = model_colors;
plot(ax2,x,comparison_table.WorstAbsoluteLateralError_m,'kx', ...
    'LineWidth',1.8,'MarkerSize',8,'DisplayName','Worst absolute error');
ylabel(ax2,'Absolute lateral error [m]');
title(ax2,'Tail and worst-case tracking error');
legend(ax2,'Location','best'); set_model_ticks(ax2,model_labels);

ax3 = nexttile(aggregate_layout); hold(ax3,'on'); grid(ax3,'on');
b3 = bar(ax3,x,comparison_table.RunBalancedMeanMPCTime_ms,0.65, ...
    'FaceColor','flat','DisplayName','Mean MPC time');
b3.CData = model_colors;
plot(ax3,x,comparison_table.MeanRunMPCTimeP95_ms,'ko', ...
    'MarkerFaceColor','w','DisplayName','Mean run P95');
plot(ax3,x,comparison_table.WorstOnlineCycleTime_ms,'kx', ...
    'LineWidth',1.8,'MarkerSize',8,'DisplayName','Worst online cycle');
ylabel(ax3,'Computation time [ms]'); title(ax3,'Online computation');
legend(ax3,'Location','best'); set_model_ticks(ax3,model_labels);

ax4 = nexttile(aggregate_layout); hold(ax4,'on'); grid(ax4,'on');
for model_idx = 1:n_models
    scatter(ax4,comparison_table.RunBalancedMeanMPCTime_ms(model_idx), ...
        comparison_table.RunBalancedLateralRMSE_m(model_idx),90, ...
        model_colors(model_idx,:),'filled','DisplayName', ...
        model_labels(model_idx));
end
xlabel(ax4,'Mean MPC computation time [ms]');
ylabel(ax4,'Equal-run lateral RMSE [m]');
title(ax4,'Accuracy-computation trade-off');
legend(ax4,'Location','best');

% Per-run matrices expose models whose aggregate score hides a weak event.
selected_controller_keys = comparison_table.ControllerKey;
scenario_rows = ismember(comparison_database.ControllerKey, ...
    selected_controller_keys);
scenario_database = comparison_database(scenario_rows,:);
run_ids = unique(scenario_database.EvaluationRun,'sorted');
run_labels = strings(numel(run_ids),1);
rmse_matrix = NaN(numel(run_ids),n_models);
deadline_matrix = NaN(numel(run_ids),n_models);
for run_idx = 1:numel(run_ids)
    representative = scenario_database( ...
        scenario_database.EvaluationRun==run_ids(run_idx),:);
    run_labels(run_idx) = sprintf('Run %d: %s / %s',run_ids(run_idx), ...
        representative.Event(1),representative.TrackLayout(1));
    for model_idx = 1:n_models
        row = scenario_database.EvaluationRun==run_ids(run_idx) & ...
            scenario_database.ControllerKey==selected_controller_keys(model_idx);
        if any(row)
            assert(nnz(row)==1,'Duplicate model/run result in database.');
            rmse_matrix(run_idx,model_idx) = ...
                scenario_database.LateralRMSE_m(row);
            deadline_matrix(run_idx,model_idx) = ...
                scenario_database.DeadlineMissPercent(row);
        end
    end
end

figure('Name','Closed-loop per-run lateral RMSE', ...
    'Position',[120 120 1300 700]);
h1 = heatmap(cellstr(model_labels),cellstr(run_labels),rmse_matrix);
h1.Title = 'Lateral RMSE by held-out run [m]';
h1.XLabel = 'Prediction model'; h1.YLabel = 'Evaluation scenario';
h1.MissingDataLabel = 'Not evaluated';

figure('Name','Closed-loop per-run deadline misses', ...
    'Position',[140 140 1300 700]);
h2 = heatmap(cellstr(model_labels),cellstr(run_labels),deadline_matrix);
h2.Title = 'Control deadline misses by held-out run [%]';
h2.XLabel = 'Prediction model'; h2.YLabel = 'Evaluation scenario';
h2.MissingDataLabel = 'Not evaluated';
%%

%% LOCAL FUNCTIONS

function selected = select_results(results,run_indices)
available = cellfun(@(r) r.evaluation_run,results);
if isempty(run_indices)
    selected = results([]);
    return
end
missing = setdiff(run_indices,available);
assert(isempty(missing), ...
    'Requested plot run(s) were not simulated: %s.',mat2str(missing));
selected = results(ismember(available,run_indices));
end

function plot_scenario_input(result)
data = result.data;
meas = result.meas;
run_idx = result.evaluation_run;
figure('Name',sprintf('Data check - run %d',run_idx), ...
    'Position',[100 100 1200 600]);
main_layout = tiledlayout(1,2);
nexttile(main_layout,1); hold on; grid on; axis equal;
plot(data.x,data.y,'k');
plot(meas.x,meas.y,'b','LineWidth',1.5);
xlabel('X [m]'); ylabel('Y [m]'); title('Trajectory map');
legend('Full run','Simulated interval');

right_layout = tiledlayout(main_layout,4,1, ...
    'TileSpacing','compact','Padding','compact');
right_layout.Layout.Tile = 2;
ax(1) = nexttile(right_layout); hold on; grid on;
plot(data.time,data.vx,'k'); plot(meas.t,meas.vx,'b','LineWidth',1.2);
ylabel('V_x [m/s]'); title('Longitudinal velocity');
ax(2) = nexttile(right_layout); hold on; grid on;
plot(data.time,data.vy,'k'); plot(meas.t,meas.vy,'b','LineWidth',1.2);
ylabel('V_y [m/s]'); title('Lateral velocity');
ax(3) = nexttile(right_layout); hold on; grid on;
plot(data.time,data.r,'k'); plot(meas.t,meas.r,'b','LineWidth',1.2);
ylabel('r [rad/s]'); title('Yaw rate');
ax(4) = nexttile(right_layout); hold on; grid on;
plot(data.time,data.delta,'k');
plot(meas.t,meas.delta,'b','LineWidth',1.2);
plot(meas.t,result.st_sim,'Color',[0.850 0.325 0.098]);
ylabel('\delta [rad]'); xlabel('Time [s]'); title('Steering');
legend('Full measured position','Interval measured position', ...
    'Simulated command');
linkaxes(ax,'x');
end

function plot_scenario_result(result)
figure('Name',sprintf('MPC Results - run %d',result.evaluation_run), ...
    'Position',[100 100 1200 800]);
tiledlayout(3,1,'TileSpacing','compact','Padding','compact');
ax1 = nexttile; hold on; grid on; axis equal;
plot(result.meas.x,result.meas.y,'k-','LineWidth',2);
plot(result.x_sim,result.y_sim,'r-','LineWidth',2);
xlabel('X [m]'); ylabel('Y [m]'); title('Global Trajectory Tracking');
legend('Reference (meas)','Simulated','Location','best');
set(ax1,'Color','w');

ax2 = nexttile; hold on; grid on;
plot(result.time,result.lateral_error,'k-','LineWidth',1.5);
yline(0,'Color',[0.5 0.5 0.5],'LineStyle','--', ...
    'HandleVisibility','off');
ylabel('e_y [m]'); xlabel('Time [s]');
title(sprintf('Signed lateral error: RMSE %.3f m, max |e_y| %.3f m', ...
    sqrt(mean(result.lateral_error.^2)), ...
    max(abs(result.lateral_error))));

ax3 = nexttile; hold on; grid on;
plot(result.time,result.model_error,'LineWidth',1.5);
ylabel('Vector norm difference'); xlabel('Time [s]');
title('Model Error'); legend('Model error','Location','best');
linkaxes([ax2 ax3],'x');
end

function colors = closed_loop_model_colors(labels)
% Keep model colours stable across every closed-loop comparison plot.
colors = zeros(numel(labels),3);
for i = 1:numel(labels)
    label = lower(string(labels(i)));
    if label=="ltv"
        colors(i,:) = [0.000 0.447 0.741];
    elseif label=="anfis_direct"
        colors(i,:) = [0.850 0.325 0.098];
    elseif label=="anfis_delta"
        colors(i,:) = [0.929 0.694 0.125];
    elseif label=="anfis_derivative"
        colors(i,:) = [0.494 0.184 0.556];
    elseif label=="anfis_ltv_residual"
        colors(i,:) = [0.466 0.674 0.188];
    elseif label=="eefig frozen"
        colors(i,:) = [0.301 0.745 0.933];
    elseif label=="eefig adaptive"
        colors(i,:) = [0.000 0.500 0.200];
    else
        colors(i,:) = [0.500 0.500 0.500];
    end
end
end

function set_model_ticks(ax,labels)
ax.XTick = 1:numel(labels);
ax.XTickLabel = cellstr(labels);
ax.XTickLabelRotation = 20;
ax.TickLabelInterpreter = 'none';
end

function database = upgrade_results_database(database,result_template)
if isempty(database)
    database = result_template([],:);
    return
end
names = database.Properties.VariableNames;
if ~ismember('DeclaredLaps',names)
    database = addvars(database,NaN(height(database),1), ...
        'After','TrackLayout','NewVariableNames','DeclaredLaps');
end
names = database.Properties.VariableNames;
if ~ismember('EvaluationProtocol',names)
    database = addvars(database, ...
        repmat("selected_window",height(database),1), ...
        'After','DatasetFile','NewVariableNames','EvaluationProtocol');
end
names = database.Properties.VariableNames;
if ~ismember('DatasetScenarioCount',names)
    database = addvars(database,NaN(height(database),1), ...
        'After','EvaluationProtocol','NewVariableNames','DatasetScenarioCount');
end
names = database.Properties.VariableNames;
if ~ismember('ControllerKey',names) || ~ismember('ScenarioKey',names)
    controller_keys = strings(height(database),1);
    scenario_keys = strings(height(database),1);
    for row = 1:height(database)
        legacy_setup = jsondecode(database.SetupKey(row));
        if ~isfield(legacy_setup,'EvaluationProtocol')
            legacy_setup.EvaluationProtocol = ...
                char(database.EvaluationProtocol(row));
            legacy_setup.TrimStartSeconds = 0;
            legacy_setup.TrimEndSeconds = 0;
            legacy_setup.MaxMPCWindowsPerRun = NaN;
            database.SetupKey(row) = string(jsonencode(legacy_setup));
        end
        legacy_controller = rmfield(legacy_setup, ...
            intersect({'EvaluationRun','WindowStart','SampleCount'}, ...
            fieldnames(legacy_setup),'stable'));
        legacy_scenario = struct( ...
            'DatasetFile',char(database.DatasetFile(row)), ...
            'EvaluationRun',database.EvaluationRun(row), ...
            'WindowStart',database.WindowStart(row), ...
            'SampleCount',database.SampleCount(row), ...
            'Event',char(database.Event(row)), ...
            'TrackLayout',char(database.TrackLayout(row)), ...
            'DeclaredLaps',database.DeclaredLaps(row));
        controller_keys(row) = string(jsonencode(legacy_controller));
        scenario_keys(row) = string(jsonencode(legacy_scenario));
    end
    database = addvars(database,controller_keys,scenario_keys, ...
        'After','SetupKey','NewVariableNames', ...
        {'ControllerKey','ScenarioKey'});
end
expected_names = result_template.Properties.VariableNames;
actual_names = database.Properties.VariableNames;
assert(isempty(setxor(expected_names,actual_names)), ...
    'The existing MPC results database has an incompatible schema.');
database = database(:,expected_names);
end

function summary = build_results_summary(database)
if isempty(database)
    summary = table();
    return
end
keys = unique(database.ControllerKey,'stable');
n = numel(keys);
model = strings(n,1); plant = strings(n,1); protocol = strings(n,1);
horizon = NaN(n,1); sample_time = NaN(n,1);
scenarios = zeros(n,1); expected = NaN(n,1); coverage = NaN(n,1);
laps = zeros(n,1); samples = zeros(n,1);
run_rmse = NaN(n,1); median_rmse = NaN(n,1); worst_rmse = NaN(n,1);
sample_rmse = NaN(n,1); run_mae = NaN(n,1); model_error = NaN(n,1);
mpc_time = NaN(n,1); worst_cycle = NaN(n,1); misses = NaN(n,1);
failures = zeros(n,1);
for i = 1:n
    group = database(database.ControllerKey==keys(i),:);
    model(i) = group.Model(1); plant(i) = group.PlantModel(1);
    protocol(i) = group.EvaluationProtocol(1);
    horizon(i) = group.HorizonSteps(1); sample_time(i) = group.Ts_s(1);
    scenarios(i) = height(group);
    expected(i) = max(group.DatasetScenarioCount,[],'omitnan');
    if isfinite(expected(i)) && expected(i)>0
        coverage(i) = 100*scenarios(i)/expected(i);
    end
    laps(i) = sum(group.DeclaredLaps,'omitnan');
    samples(i) = sum(group.SampleCount,'omitnan');
    run_rmse(i) = mean(group.LateralRMSE_m,'omitnan');
    median_rmse(i) = median(group.LateralRMSE_m,'omitnan');
    worst_rmse(i) = max(group.LateralRMSE_m,[],'omitnan');
    valid = isfinite(group.LateralRMSE_m) & group.SampleCount>0;
    if any(valid)
        weights = group.SampleCount(valid);
        sample_rmse(i) = sqrt(sum(weights.* ...
            group.LateralRMSE_m(valid).^2)/sum(weights));
    end
    run_mae(i) = mean(group.LateralMAE_m,'omitnan');
    model_error(i) = mean(group.MeanModelError,'omitnan');
    mpc_time(i) = mean(group.MeanMPCTime_ms,'omitnan');
    worst_cycle(i) = max(group.MaxOnlineCycleTime_ms,[],'omitnan');
    misses(i) = mean(group.DeadlineMissPercent,'omitnan');
    failures(i) = sum(group.SolverFailures,'omitnan');
end
summary = table(keys,model,plant,protocol,horizon,sample_time,scenarios, ...
    expected,coverage,laps,samples,run_rmse,median_rmse,worst_rmse, ...
    sample_rmse,run_mae,model_error,mpc_time,worst_cycle,misses,failures, ...
    'VariableNames',{'ControllerKey','Model','PlantModel', ...
    'EvaluationProtocol','HorizonSteps','Ts_s','ScenarioCount', ...
    'ExpectedScenarioCount','ScenarioCoveragePercent','DeclaredLaps', ...
    'TotalSamples','RunBalancedLateralRMSE_m','MedianRunLateralRMSE_m', ...
    'WorstRunLateralRMSE_m','SampleWeightedLateralRMSE_m', ...
    'RunBalancedLateralMAE_m','RunBalancedMeanModelError', ...
    'RunBalancedMeanMPCTime_ms','WorstOnlineCycleTime_ms', ...
    'RunBalancedDeadlineMissPercent','TotalSolverFailures'});
end
