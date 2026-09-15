%% Train ANFIS models for the LTV one-step lateral-state residual
% Inputs:  [vy, r, vx, delta]
% Outputs: [e_vy, e_r], where e=x_measured(k+1)-x_LTV(k+1)
cd('/home/andreu/ros_ws/src/as/control/fuzzy_mpc/mtlb'); addpath(genpath('/home/andreu/ros_ws/src/as/control/fuzzy_mpc/mtlb'))
clear

keep_factor = 5;
filter_order = 3;
filter_window = 21;
max_samples_per_training_run = 1000;
training_dataset_file = fullfile("data","datasets_training.mat");
model_file = fullfile("models","anfis_residuals","anfis_residuals.mat");
rollout_horizons = [1 10 30 60];
rollout_stride = 20;
gain_candidates = [0 0.05 0.1 0.2 0.35 0.5 0.75 1.0];
maximum_local_lateral_radius = 1.0;

% Split complete training runs deterministically within each event/layout
% group. The final evaluation dataset is never used for fitting, checking,
% gain selection, or stability selection.
training_source = load(training_dataset_file,'datasets','meta');
[fit_run_indices,checking_run_indices] = grouped_training_split( ...
    training_source.datasets,0.25);
fprintf('Residual ANFIS fit runs: %s\n',mat2str(fit_run_indices));
fprintf('Residual ANFIS internal checking runs: %s\n', ...
    mat2str(checking_run_indices));

prepared = prepare_anfis_training_data(training_dataset_file, ...
    training_dataset_file,'residual',keep_factor,filter_order, ...
    filter_window,max_samples_per_training_run, ...
    'TrainingRunIndices',fit_run_indices, ...
    'ValidationRunIndices',checking_run_indices);
Xn_train = prepared.Xn_train;
Y_train = prepared.Y_train;
Xn_val = prepared.Xn_val;
Y_val = prepared.Y_val;

anfis_residuals = struct();
anfis_residuals.Ts = prepared.Ts;
anfis_residuals.norm.mu = prepared.mu;
anfis_residuals.norm.sigma = prepared.sigma;
anfis_residuals.norm.x_min = prepared.x_min;
anfis_residuals.norm.x_max = prepared.x_max;
anfis_residuals.vy.min = min(Y_train(:,1));
anfis_residuals.vy.max = max(Y_train(:,1));
anfis_residuals.r.min = min(Y_train(:,2));
anfis_residuals.r.max = max(Y_train(:,2));
anfis_residuals.training.dataset_file = training_dataset_file;
anfis_residuals.training.validation_dataset_file = training_dataset_file;
anfis_residuals.training.fit_run_indices = fit_run_indices;
anfis_residuals.training.checking_run_indices = checking_run_indices;
anfis_residuals.training.final_evaluation_used = false;
anfis_residuals.training.keep_factor = keep_factor;
anfis_residuals.training.filter_order = filter_order;
anfis_residuals.training.filter_window = filter_window;
anfis_residuals.training.max_samples_per_run = max_samples_per_training_run;
anfis_residuals.training.training_run_id = prepared.training_run_id;
anfis_residuals.training.validation_run_id = prepared.validation_run_id;
save(model_file,'anfis_residuals')

%% e_vy model
opt = genfisOptions("SubtractiveClustering");
    % GridPartition:
    % opt.NumMembershipFunctions = [2 2 2 2 3];
    % opt.InputMembershipFunctionType = "gaussmf"; % gbellmf gaussmf trimf trapmf dsigmf psigmf pimf

    % SubtractiveClustering: 
    opt.ClusterInfluenceRange = 0.35; %0.5

anfis_residuals.vy.init_fis = genfis(Xn_train,Y_train(:,1),opt);

% Training options
opt = anfisOptions;
opt.InitialFIS = anfis_residuals.vy.init_fis;
opt.ValidationData = [Xn_val Y_val(:,1)];

opt.EpochNumber = 200;
opt.InitialStepSize = 0.15; %0.01
opt.StepSizeDecreaseRate = 0.9; %0.9
opt.StepSizeIncreaseRate = 1.1; %1.1

opt.DisplayErrorValues = true;
opt.DisplayStepSize    = true;
opt.DisplayANFISInformation = true;
opt.DisplayFinalResults = true;

[vy_final_fis,trainError,~,vy_validation_fis,valError] = ...
    anfis([Xn_train Y_train(:,1)],opt);
anfis_residuals.vy.fis=vy_validation_fis;
anfis_residuals.vy.final_epoch_fis=vy_final_fis;
anfis_residuals.vy.train_error=trainError;
anfis_residuals.vy.validation_error=valError;
save(model_file,'anfis_residuals')

%% e_r model
opt = genfisOptions("SubtractiveClustering");
    % GridPartition:
    % opt.NumMembershipFunctions = [2 2 2 2 3];
    % opt.InputMembershipFunctionType = "gaussmf"; % gbellmf gaussmf trimf trapmf dsigmf psigmf pimf

    % SubtractiveClustering: 
    opt.ClusterInfluenceRange = 0.35; %0.5

anfis_residuals.r.init_fis = genfis(Xn_train, Y_train(:,2), opt);

% Training options
opt = anfisOptions;
opt.InitialFIS = anfis_residuals.r.init_fis;
opt.ValidationData = [Xn_val Y_val(:,2)];

opt.EpochNumber = 200;
opt.InitialStepSize = 0.15; %0.01
opt.StepSizeDecreaseRate = 0.9; %0.9
opt.StepSizeIncreaseRate = 1.1; %1.1

opt.DisplayErrorValues = true;
opt.DisplayStepSize    = true;
opt.DisplayANFISInformation = true;
opt.DisplayFinalResults = true;

[r_final_fis,trainError,~,r_validation_fis,valError] = ...
    anfis([Xn_train Y_train(:,2)],opt);
anfis_residuals.r.fis=r_validation_fis;
anfis_residuals.r.final_epoch_fis=r_final_fis;
anfis_residuals.r.train_error=trainError;
anfis_residuals.r.validation_error=valError;

% Save model
save(model_file,'anfis_residuals')

% Training log:    
% - SC: Clusters:0.35, epoch:200, init:0.15, dec:0.9, inc:1.1 -> 0.144425, 9 rules, 67% RMSE inicial


%% Extract and save matrices

anfis_residuals.vy.mat = extract_fis(anfis_residuals.vy.fis);
anfis_residuals.r.mat  = extract_fis(anfis_residuals.r.fis);

%% Training-only recursive calibration and stability acceptance

checking_runs = build_rollout_runs( ...
    training_source.datasets(checking_run_indices), ...
    rollout_horizons,rollout_stride);
gain_results = evaluate_residual_gains(anfis_residuals,checking_runs, ...
    gain_candidates,prepared.Ts,rollout_horizons);
local_radius = residual_local_radius(anfis_residuals,checking_runs, ...
    gain_candidates,prepared.Ts);

base_vy = gain_results.RunRMSE_vy(1);
base_r = gain_results.RunRMSE_r(1);
improves_both = gain_results.RunRMSE_vy < base_vy & ...
                gain_results.RunRMSE_r < base_r;
stable = local_radius <= maximum_local_lateral_radius;
feasible = gain_candidates(:)>0 & improves_both & stable;

if any(feasible)
    relative_score = max([gain_results.RunRMSE_vy/base_vy, ...
                          gain_results.RunRMSE_r/base_r],[],2);
    relative_score(~feasible) = inf;
    [~,selected_idx] = min(relative_score);
else
    selected_idx = 1;
    warning(['No nonzero residual gain improved both states at 60 steps ' ...
        'while satisfying the local stability limit. Using plain LTV ' ...
        '(residual gain zero).']);
end

anfis_residuals.residual_gain = gain_candidates(selected_idx);
gain_results.MaximumLocalLateralRadius = local_radius;
gain_results.ImprovesBothStates = improves_both;
gain_results.SatisfiesStabilityLimit = stable;
gain_results.Selected = false(height(gain_results),1);
gain_results.Selected(selected_idx) = true;
anfis_residuals.training.rollout_horizons = rollout_horizons;
anfis_residuals.training.rollout_stride = rollout_stride;
anfis_residuals.training.maximum_local_lateral_radius = ...
    maximum_local_lateral_radius;
anfis_residuals.training.gain_selection = gain_results;

fprintf('\nResidual-gain selection on internal training-run checking split:\n');
disp(gain_results);
fprintf('Selected residual gain: %.3g\n',anfis_residuals.residual_gain);

save(model_file,'anfis_residuals')


%% Model insights

% Model to evaluate
fis = anfis_residuals.r.fis;
insight_figures = anfis_model_insights(fis,Xn_train,trainError,valError, ...
    'inputLabels', {'vy','r','vx','delta'}, ...
    'titlePrefix', 'anfis\_residuals.r', ...
    'figBase', 0);
save_script_figures('anfis_residuals_training',insight_figures);

function [fit_indices,checking_indices] = grouped_training_split( ...
    datasets,checking_fraction)
event = string({datasets.event});
track = strings(size(event));
for idx = 1:numel(datasets)
    if isfield(datasets(idx),'track_layout') && ...
            ~isempty(datasets(idx).track_layout)
        track(idx) = string(datasets(idx).track_layout);
    elseif isfield(datasets(idx),'track_id')
        track(idx) = "track_" + string(datasets(idx).track_id);
    else
        track(idx) = "unspecified";
    end
end
group = event + "|" + track;
checking_indices = [];
groups = unique(group,'stable');
for group_idx = 1:numel(groups)
    members = find(group==groups(group_idx));
    if numel(members)<2
        continue;
    end
    checking_count = max(1,round(checking_fraction*numel(members)));
    checking_count = min(checking_count,numel(members)-1);
    positions = unique(round(linspace(2,numel(members),checking_count)));
    checking_indices = [checking_indices,members(positions)]; %#ok<AGROW>
end
checking_indices = unique(checking_indices,'stable');
fit_indices = setdiff(1:numel(datasets),checking_indices,'stable');
if isempty(checking_indices) || isempty(fit_indices)
    error('Could not form nonempty fit and checking run groups.');
end
end

function runs = build_rollout_runs(datasets,horizons,stride)
maximum_horizon = max(horizons);
runs = repmat(struct('vy',[],'r',[],'vx',[],'delta',[], ...
    'origins',[]),numel(datasets),1);
for run_idx = 1:numel(datasets)
    data = datasets(run_idx).data;
    count = min([numel(data.vy),numel(data.r),numel(data.vx), ...
                 numel(data.delta)]);
    ini = datasets(run_idx).ini;
    fin = datasets(run_idx).fin;
    if isempty(ini),ini=1;end
    if isempty(fin),fin=count;end
    ini = max(1,ini(1));
    fin = min(count,fin(end));
    runs(run_idx).vy = data.vy(:);
    runs(run_idx).r = data.r(:);
    runs(run_idx).vx = data.vx(:);
    runs(run_idx).delta = data.delta(:);
    candidates = (ini:stride:(fin-maximum_horizon))';
    valid = false(size(candidates));
    for candidate_idx = 1:numel(candidates)
        sequence = candidates(candidate_idx)+(0:maximum_horizon);
        valid(candidate_idx) = all(isfinite(data.vy(sequence))) && ...
            all(isfinite(data.r(sequence))) && ...
            all(isfinite(data.vx(sequence(1:end-1)))) && ...
            all(isfinite(data.delta(sequence(1:end-1))));
    end
    runs(run_idx).origins = candidates(valid);
    if isempty(runs(run_idx).origins)
        error('Internal checking run %d has no valid rollout origins.',run_idx);
    end
end
end

function results = evaluate_residual_gains(model,runs,gains,Ts,horizons)
run_rmse_vy = zeros(numel(gains),numel(runs));
run_rmse_r = zeros(numel(gains),numel(runs));
sample_errors_vy = cell(numel(gains),1);
sample_errors_r = cell(numel(gains),1);
for gain_idx = 1:numel(gains)
    gain = gains(gain_idx);
    for run_idx = 1:numel(runs)
        run = runs(run_idx);
        error_vy = zeros(numel(run.origins),1);
        error_r = zeros(numel(run.origins),1);
        for origin_idx = 1:numel(run.origins)
            origin = run.origins(origin_idx);
            state = [run.vy(origin);run.r(origin)];
            for step = 1:max(horizons)
                sample = origin+step-1;
                state = residual_lateral_step(model,state,run.vx(sample), ...
                    run.delta(sample),Ts,gain);
            end
            target = origin+max(horizons);
            error_vy(origin_idx) = state(1)-run.vy(target);
            error_r(origin_idx) = state(2)-run.r(target);
        end
        run_rmse_vy(gain_idx,run_idx) = sqrt(mean(error_vy.^2));
        run_rmse_r(gain_idx,run_idx) = sqrt(mean(error_r.^2));
        sample_errors_vy{gain_idx} = [sample_errors_vy{gain_idx};error_vy];
        sample_errors_r{gain_idx} = [sample_errors_r{gain_idx};error_r];
    end
end
sample_rmse_vy = cellfun(@(e)sqrt(mean(e.^2)),sample_errors_vy);
sample_rmse_r = cellfun(@(e)sqrt(mean(e.^2)),sample_errors_r);
results = table(gains(:),sample_rmse_vy,sample_rmse_r, ...
    mean(run_rmse_vy,2),mean(run_rmse_r,2), ...
    'VariableNames',{'Gain','SampleRMSE_vy','SampleRMSE_r', ...
    'RunRMSE_vy','RunRMSE_r'});
end

function next = residual_lateral_step(model,state,vx,delta,Ts,gain)
full_state = [0;state(1);0;state(2);delta;0];
base = ltv(full_state,[vx;delta],Ts);
input = [state(1),state(2),vx,delta];
input = min(max(input,model.norm.x_min),model.norm.x_max);
input_n = (input-model.norm.mu)./model.norm.sigma;
[~,~,correction_vy] = evalfis_mat(model.vy.mat,input_n);
[~,~,correction_r] = evalfis_mat(model.r.mat,input_n);
next = base([2 4])+gain*[correction_vy;correction_r];
end

function maximum_radius = residual_local_radius(model,runs,gains,Ts)
maximum_radius = zeros(numel(gains),1);
for run_idx = 1:numel(runs)
    run = runs(run_idx);
    sample_indices = unique(round(linspace(1,numel(run.vx), ...
        min(200,numel(run.vx)))));
    for sample = sample_indices
        state = [run.vy(sample);run.r(sample)];
        epsilon = 1e-5;
        jacobian_base = zeros(2);
        jacobian_full = zeros(2);
        for state_idx = 1:2
            perturbation = zeros(2,1);
            perturbation(state_idx) = epsilon;
            plus_base = residual_lateral_step(model,state+perturbation, ...
                run.vx(sample),run.delta(sample),Ts,0);
            minus_base = residual_lateral_step(model,state-perturbation, ...
                run.vx(sample),run.delta(sample),Ts,0);
            plus_full = residual_lateral_step(model,state+perturbation, ...
                run.vx(sample),run.delta(sample),Ts,1);
            minus_full = residual_lateral_step(model,state-perturbation, ...
                run.vx(sample),run.delta(sample),Ts,1);
            jacobian_base(:,state_idx) = ...
                (plus_base-minus_base)/(2*epsilon);
            jacobian_full(:,state_idx) = ...
                (plus_full-minus_full)/(2*epsilon);
        end
        correction_jacobian = jacobian_full-jacobian_base;
        for gain_idx = 1:numel(gains)
            radius = max(abs(eig(jacobian_base+ ...
                gains(gain_idx)*correction_jacobian)));
            maximum_radius(gain_idx) = max(maximum_radius(gain_idx),radius);
        end
    end
end
end
