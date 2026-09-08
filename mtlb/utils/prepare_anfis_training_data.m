function prepared = prepare_anfis_training_data(training_dataset_file, ...
    validation_dataset_file, target_type, keep_factor, filter_order, ...
    filter_window, max_samples_per_training_run)
%PREPARE_ANFIS_TRAINING_DATA Build leakage-free ANFIS train/checking data.
% Inputs are [vy r vx delta]. target_type is direct, dot, delta, or residual.

target_type = validatestring(target_type, ...
    {'direct','dot','delta','residual'}, mfilename, 'target_type');
validateattributes(keep_factor, {'numeric'}, ...
    {'scalar','integer','positive'}, mfilename, 'keep_factor');
validateattributes(filter_order, {'numeric'}, ...
    {'scalar','integer','nonnegative'}, mfilename, 'filter_order');
validateattributes(filter_window, {'numeric'}, ...
    {'scalar','integer','positive'}, mfilename, 'filter_window');
validateattributes(max_samples_per_training_run, {'numeric'}, ...
    {'scalar','integer','positive'}, mfilename, ...
    'max_samples_per_training_run');

if ~isfile(training_dataset_file)
    error('Training dataset not found: %s', training_dataset_file);
end
if ~isfile(validation_dataset_file)
    error('Validation dataset not found: %s', validation_dataset_file);
end

training_source = load(training_dataset_file, 'datasets', 'meta');
validation_source = load(validation_dataset_file, 'datasets', 'meta');
Ts = training_source.meta.Ts;
if abs(validation_source.meta.Ts-Ts) > ...
        eps(max(validation_source.meta.Ts,Ts))
    error('Training Ts (%g s) and validation Ts (%g s) do not match.', ...
        Ts,validation_source.meta.Ts);
end

assert_independent_runs(training_source.datasets, ...
    validation_source.datasets);
training_runs = build_run_samples(training_source.datasets,target_type,Ts, ...
    keep_factor,filter_order,filter_window,"training");
validation_runs = build_run_samples(validation_source.datasets,target_type,Ts, ...
    keep_factor,filter_order,filter_window,"validation");

[X_train,Y_train,training_run_id] = concatenate_runs(training_runs, ...
    max_samples_per_training_run);
[X_val,Y_val,validation_run_id] = concatenate_runs(validation_runs,[]);

mu = mean(X_train,1);
sigma = std(X_train,0,1);
if any(~isfinite(mu)) || any(~isfinite(sigma)) || any(sigma<=1e-8)
    error('Invalid normalization: one or more training inputs have near-zero variance.');
end

prepared.X_train = X_train;
prepared.Y_train = Y_train;
prepared.X_val = X_val;
prepared.Y_val = Y_val;
prepared.Xn_train = (X_train-mu)./sigma;
prepared.Xn_val = (X_val-mu)./sigma;
prepared.mu = mu;
prepared.sigma = sigma;
prepared.x_min = min(X_train,[],1);
prepared.x_max = max(X_train,[],1);
prepared.Ts = Ts;
prepared.training_run_id = training_run_id;
prepared.validation_run_id = validation_run_id;
prepared.training_run_count = numel(training_runs);
prepared.validation_run_count = numel(validation_runs);

fprintf('Training:   %d points from %d independent runs.\n', ...
    size(X_train,1),prepared.training_run_count);
fprintf('Validation: %d points from %d held-out runs.\n', ...
    size(X_val,1),prepared.validation_run_count);
end

function runs = build_run_samples(datasets,target_type,Ts,keep_factor, ...
    filter_order,filter_window,split_name)
required_signals = {'vy','r','vx','delta'};
if strcmp(target_type,'dot')
    required_signals{end+1} = 'ay';
end
runs = repmat(struct('X',[],'Y',[]),numel(datasets),1);

for run_idx = 1:numel(datasets)
    data = datasets(run_idx).data;
    lengths = zeros(size(required_signals));
    for signal_idx = 1:numel(required_signals)
        signal_name = required_signals{signal_idx};
        if ~isfield(data,signal_name)
            error('%s run %d is missing signal data.%s.', ...
                split_name,run_idx,signal_name);
        end
        lengths(signal_idx) = numel(data.(signal_name));
    end
    if any(lengths~=lengths(1))
        error('%s run %d has signals with inconsistent lengths.', ...
            split_name,run_idx);
    end

    sample_count = lengths(1);
    ini = datasets(run_idx).ini;
    fin = datasets(run_idx).fin;
    if isempty(ini), ini=1; end
    if isempty(fin), fin=sample_count; end
    validateattributes(ini,{'numeric'}, ...
        {'scalar','integer','>=',1},mfilename,'ini');
    validateattributes(fin,{'numeric'}, ...
        {'scalar','integer','<=',sample_count},mfilename,'fin');
    if fin<=ini
        error('%s run %d needs at least two selected samples.', ...
            split_name,run_idx);
    end
    if fin-ini+1<filter_window
        error('%s run %d is shorter than the %d-sample filter window.', ...
            split_name,run_idx,filter_window);
    end

    selected = ini:fin;
    filtered = struct();
    for signal_idx = 1:numel(required_signals)
        signal_name = required_signals{signal_idx};
        values = data.(signal_name);
        filtered_signal = sgolayfilt_custom( ...
            values(selected),filter_order,filter_window);
        filtered.(signal_name) = filtered_signal(:);
    end

    idx = (1:keep_factor:(numel(selected)-1))';
    idx_next = idx+1;
    X = [filtered.vy(idx),filtered.r(idx),filtered.vx(idx), ...
        filtered.delta(idx)];

    switch target_type
        case 'direct'
            Y = [filtered.vy(idx_next),filtered.r(idx_next)];
        case 'delta'
            Y = [filtered.vy(idx_next)-filtered.vy(idx), ...
                filtered.r(idx_next)-filtered.r(idx)];
        case 'dot'
            Y = [filtered.ay(idx)-filtered.vx(idx).*filtered.r(idx), ...
                (filtered.r(idx_next)-filtered.r(idx))./Ts];
        case 'residual'
            Y = residual_targets(X,filtered.vy(idx_next), ...
                filtered.r(idx_next),Ts);
    end

    valid = all(isfinite(X),2) & all(isfinite(Y),2);
    runs(run_idx).X = X(valid,:);
    runs(run_idx).Y = Y(valid,:);
    if isempty(runs(run_idx).X)
        error('%s run %d contains no finite transitions.',split_name,run_idx);
    end
    fprintf('%s run %d prepared: %d transitions.\n', ...
        split_name,run_idx,size(runs(run_idx).X,1));
end
end

function Y = residual_targets(X,vy_next,r_next,Ts)
Y = zeros(size(X,1),2);
for sample_idx = 1:size(X,1)
    state = [0;X(sample_idx,1);0;X(sample_idx,2);X(sample_idx,4);0];
    % The command affects steering acceleration, not the current one-step
    % lateral prediction, so its value is immaterial for these two targets.
    prediction = ltv(state,[X(sample_idx,3),X(sample_idx,4)],Ts);
    Y(sample_idx,:) = [vy_next(sample_idx)-prediction(2), ...
        r_next(sample_idx)-prediction(4)];
end
end

function [X,Y,run_id] = concatenate_runs(runs,max_samples_per_run)
X_parts = cell(numel(runs),1);
Y_parts = cell(numel(runs),1);
id_parts = cell(numel(runs),1);
for run_idx = 1:numel(runs)
    count = size(runs(run_idx).X,1);
    if ~isempty(max_samples_per_run) && count>max_samples_per_run
        idx = unique(round(linspace(1,count,max_samples_per_run)));
    else
        idx = 1:count;
    end
    X_parts{run_idx} = runs(run_idx).X(idx,:);
    Y_parts{run_idx} = runs(run_idx).Y(idx,:);
    id_parts{run_idx} = repmat(run_idx,numel(idx),1);
end
X = vertcat(X_parts{:});
Y = vertcat(Y_parts{:});
run_id = vertcat(id_parts{:});
end

function assert_independent_runs(training_datasets,validation_datasets)
training_paths = string({training_datasets.path});
validation_paths = string({validation_datasets.path});
overlap = intersect(training_paths,validation_paths);
if ~isempty(overlap)
    overlap_text = strjoin(cellstr(overlap),sprintf('\n  '));
    error(['Training and validation must contain independent runs. ' ...
        'These paths occur in both datasets:\n  %s'],overlap_text);
end
end
