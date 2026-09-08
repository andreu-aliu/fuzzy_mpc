%% Train ANFIS models for one-step lateral-state increments
% Inputs:  [vy, r, vx, delta]
% Outputs: [vy(k+1)-vy(k), r(k+1)-r(k)]

clear;
clc;

mtlb_dir = '/home/andreu/ros_ws/src/as/control/fuzzy_mpc/mtlb';
cd(mtlb_dir);
addpath(genpath(mtlb_dir));

%% Configuration

training_dataset_file = fullfile("data", "datasets_training.mat");
validation_dataset_file = fullfile("data", "datasets_evaluation.mat");
model_file = fullfile("models", "anfis_delta", "anfis_delta.mat");

keep_factor = 5;       % Keep one transition out of every keep_factor samples.
filter_order = 3;
filter_window = 21;
max_samples_per_training_run = 1000; % Cap long runs; never repeat short runs.

%% Load the independent training and validation runs

if ~isfile(training_dataset_file)
    error('Training dataset not found: %s', training_dataset_file);
end
if ~isfile(validation_dataset_file)
    error('Validation dataset not found: %s', validation_dataset_file);
end

training_source = load(training_dataset_file, 'datasets', 'meta');
validation_source = load(validation_dataset_file, 'datasets', 'meta');

Ts = training_source.meta.Ts;
if abs(validation_source.meta.Ts - Ts) > eps(max(Ts, validation_source.meta.Ts))
    error('Training Ts (%g s) and validation Ts (%g s) do not match.', ...
        Ts, validation_source.meta.Ts);
end

assert_independent_runs(training_source.datasets, validation_source.datasets);

training_runs = build_run_samples(training_source.datasets, keep_factor, ...
    filter_order, filter_window, "training");
validation_runs = build_run_samples(validation_source.datasets, keep_factor, ...
    filter_order, filter_window, "validation");

[X_train, Y_train, training_run_id] = concatenate_runs( ...
    training_runs,max_samples_per_training_run);
[X_val, Y_val, validation_run_id] = concatenate_runs(validation_runs,[]);

fprintf('Training:   %d points from %d independent runs.\n', ...
    size(X_train, 1), numel(training_runs));
fprintf('Validation: %d points from %d held-out runs.\n', ...
    size(X_val, 1), numel(validation_runs));

%% Normalize using training data only

mu = mean(X_train, 1);
sigma = std(X_train, 0, 1);
if any(~isfinite(mu)) || any(~isfinite(sigma)) || any(sigma <= 1e-8)
    error('Invalid training normalization: one or more inputs have near-zero variance.');
end

Xn_train = (X_train - mu) ./ sigma;
Xn_val = (X_val - mu) ./ sigma;

anfis_delta = struct();
anfis_delta.Ts = Ts;
anfis_delta.norm.mu = mu;
anfis_delta.norm.sigma = sigma;
anfis_delta.norm.x_min = min(X_train, [], 1);
anfis_delta.norm.x_max = max(X_train, [], 1);
anfis_delta.vy.min = min(Y_train(:,1));
anfis_delta.vy.max = max(Y_train(:,1));
anfis_delta.r.min = min(Y_train(:,2));
anfis_delta.r.max = max(Y_train(:,2));
anfis_delta.training.dataset_file = training_dataset_file;
anfis_delta.training.validation_dataset_file = validation_dataset_file;
anfis_delta.training.keep_factor = keep_factor;
anfis_delta.training.filter_order = filter_order;
anfis_delta.training.filter_window = filter_window;
anfis_delta.training.max_samples_per_run = max_samples_per_training_run;
anfis_delta.training.training_run_id = training_run_id;
anfis_delta.training.validation_run_id = validation_run_id;

%% Train vy-increment model

fis_options = genfisOptions("SubtractiveClustering");
fis_options.ClusterInfluenceRange = 0.35;
anfis_delta.vy.init_fis = genfis(Xn_train, Y_train(:,1), fis_options);

train_options = anfisOptions;
train_options.InitialFIS = anfis_delta.vy.init_fis;
train_options.ValidationData = [Xn_val Y_val(:,1)];
train_options.EpochNumber = 200;
train_options.InitialStepSize = 0.15;
train_options.StepSizeDecreaseRate = 0.9;
train_options.StepSizeIncreaseRate = 1.1;
train_options.DisplayErrorValues = true;
train_options.DisplayStepSize = true;
train_options.DisplayANFISInformation = true;
train_options.DisplayFinalResults = true;

[vy_final_fis, vy_train_error, ~, vy_validation_fis, vy_val_error] = ...
    anfis([Xn_train Y_train(:,1)], train_options);

% The checking-data FIS is the epoch with the lowest held-out error.
anfis_delta.vy.fis = vy_validation_fis;
anfis_delta.vy.final_epoch_fis = vy_final_fis;
anfis_delta.vy.train_error = vy_train_error;
anfis_delta.vy.validation_error = vy_val_error;

%% Train yaw-rate-increment model

fis_options = genfisOptions("SubtractiveClustering");
fis_options.ClusterInfluenceRange = 0.30;
anfis_delta.r.init_fis = genfis(Xn_train, Y_train(:,2), fis_options);

train_options = anfisOptions;
train_options.InitialFIS = anfis_delta.r.init_fis;
train_options.ValidationData = [Xn_val Y_val(:,2)];
train_options.EpochNumber = 200;
train_options.InitialStepSize = 0.10;
train_options.StepSizeDecreaseRate = 0.9;
train_options.StepSizeIncreaseRate = 1.1;
train_options.DisplayErrorValues = true;
train_options.DisplayStepSize = true;
train_options.DisplayANFISInformation = true;
train_options.DisplayFinalResults = true;

[r_final_fis, r_train_error, ~, r_validation_fis, r_val_error] = ...
    anfis([Xn_train Y_train(:,2)], train_options);

anfis_delta.r.fis = r_validation_fis;
anfis_delta.r.final_epoch_fis = r_final_fis;
anfis_delta.r.train_error = r_train_error;
anfis_delta.r.validation_error = r_val_error;

%% Extract the local Takagi-Sugeno matrices and save

anfis_delta.vy.mat = extract_fis(anfis_delta.vy.fis);
anfis_delta.r.mat = extract_fis(anfis_delta.r.fis);
save(model_file, 'anfis_delta');

fprintf('Saved held-out-validation ANFIS model to %s\n', model_file);

%% Model insights

anfis_model_insights(anfis_delta.r.fis, Xn_train, ...
    r_train_error, r_val_error, ...
    'inputLabels', {'vy','r','vx','delta'}, ...
    'titlePrefix', 'anfis\_delta.r', ...
    'figBase', 0);

%% Local functions

function runs = build_run_samples(datasets, keep_factor, filter_order, ...
        filter_window, split_name)
arguments
    datasets
    keep_factor (1,1) double {mustBeInteger, mustBePositive}
    filter_order (1,1) double {mustBeInteger, mustBeNonnegative}
    filter_window (1,1) double {mustBeInteger, mustBePositive}
    split_name (1,1) string
end

required_signals = {'vy', 'r', 'vx', 'delta'};
runs = repmat(struct('X', [], 'Y', []), numel(datasets), 1);

for run_idx = 1:numel(datasets)
    data = datasets(run_idx).data;
    lengths = zeros(size(required_signals));
    for signal_idx = 1:numel(required_signals)
        signal_name = required_signals{signal_idx};
        if ~isfield(data, signal_name)
            error('%s run %d is missing signal data.%s.', ...
                split_name, run_idx, signal_name);
        end
        lengths(signal_idx) = numel(data.(signal_name));
    end
    if any(lengths ~= lengths(1))
        error('%s run %d has signals with inconsistent lengths.', ...
            split_name, run_idx);
    end

    sample_count = lengths(1);
    ini = datasets(run_idx).ini;
    fin = datasets(run_idx).fin;
    if isempty(ini), ini = 1; end
    if isempty(fin), fin = sample_count; end

    validateattributes(ini, {'numeric'}, ...
        {'scalar','real','finite','integer','>=',1}, mfilename, 'ini');
    validateattributes(fin, {'numeric'}, ...
        {'scalar','real','finite','integer','<=',sample_count}, mfilename, 'fin');
    if fin <= ini
        error('%s run %d needs at least two samples (ini=%d, fin=%d).', ...
            split_name, run_idx, ini, fin);
    end
    if fin - ini + 1 < filter_window
        error(['%s run %d has %d selected samples, fewer than the ' ...
            '%d-sample filter window.'], split_name, run_idx, ...
            fin - ini + 1, filter_window);
    end

    selected = ini:fin;
    filtered = struct();
    for signal_idx = 1:numel(required_signals)
        signal_name = required_signals{signal_idx};
        signal = data.(signal_name);
        filtered.(signal_name) = sgolayfilt_custom( ...
            signal(selected), filter_order, filter_window);
        filtered.(signal_name) = filtered.(signal_name)(:);
    end

    idx = (1:keep_factor:(numel(selected)-1))';
    idx_next = idx + 1;
    X = [filtered.vy(idx),filtered.r(idx),filtered.vx(idx), ...
        filtered.delta(idx)];
    Y = [filtered.vy(idx_next) - filtered.vy(idx), ...
        filtered.r(idx_next) - filtered.r(idx)];

    valid = all(isfinite(X), 2) & all(isfinite(Y), 2);
    runs(run_idx).X = X(valid,:);
    runs(run_idx).Y = Y(valid,:);
    if isempty(runs(run_idx).X)
        error('%s run %d contains no finite training transitions.', ...
            split_name, run_idx);
    end

    fprintf('%s run %d prepared: %d transitions.\n', ...
        split_name, run_idx, size(runs(run_idx).X, 1));
end
end

function [X,Y,run_id] = concatenate_runs(runs,max_samples_per_run)
counts = arrayfun(@(run) size(run.X, 1), runs);
if ~isempty(max_samples_per_run)
    validateattributes(max_samples_per_run, {'numeric'}, ...
        {'scalar','real','finite','integer','positive'}, ...
        mfilename, 'max_samples_per_run');
end

X_parts = cell(numel(runs), 1);
Y_parts = cell(numel(runs), 1);
id_parts = cell(numel(runs), 1);
for run_idx = 1:numel(runs)
    if ~isempty(max_samples_per_run) && counts(run_idx) > max_samples_per_run
        % Uniformly cap long runs without duplicating samples from short runs.
        idx = unique(round(linspace(1,counts(run_idx), ...
            max_samples_per_run)));
    else
        idx = 1:counts(run_idx);
    end
    X_parts{run_idx} = runs(run_idx).X(idx,:);
    Y_parts{run_idx} = runs(run_idx).Y(idx,:);
    id_parts{run_idx} = repmat(run_idx, numel(idx), 1);
end

X = vertcat(X_parts{:});
Y = vertcat(Y_parts{:});
run_id = vertcat(id_parts{:});
end

function assert_independent_runs(training_datasets, validation_datasets)
training_paths = string({training_datasets.path});
validation_paths = string({validation_datasets.path});
overlap = intersect(training_paths, validation_paths);
if ~isempty(overlap)
    overlap_text = strjoin(cellstr(overlap), sprintf('\n  '));
    error(['Training and validation must contain independent runs. ' ...
        'These paths occur in both datasets:\n  %s'], overlap_text);
end
end
