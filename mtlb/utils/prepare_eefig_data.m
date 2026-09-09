function prepared = prepare_eefig_data(training_dataset_file, ...
    evaluation_dataset_file, keep_factor, filter_order, filter_window)
%PREPARE_EEFIG_DATA Build run-separated lateral EEFIG transitions.
% State: [vy; r], scheduling/input vector: [vx; delta].

validateattributes(keep_factor, {'numeric'}, ...
    {'scalar','integer','positive'}, mfilename, 'keep_factor');
validateattributes(filter_order, {'numeric'}, ...
    {'scalar','integer','nonnegative'}, mfilename, 'filter_order');
validateattributes(filter_window, {'numeric'}, ...
    {'scalar','integer','positive','odd'}, mfilename, 'filter_window');

if ~isfile(training_dataset_file)
    error('Training dataset not found: %s', training_dataset_file);
end
if ~isfile(evaluation_dataset_file)
    error('Evaluation dataset not found: %s', evaluation_dataset_file);
end

training_source = load(training_dataset_file, 'datasets', 'meta');
evaluation_source = load(evaluation_dataset_file, 'datasets', 'meta');
assertIndependentRuns(training_source.datasets, evaluation_source.datasets);

Ts = training_source.meta.Ts;
if abs(evaluation_source.meta.Ts - Ts) > eps(max(Ts, evaluation_source.meta.Ts))
    error('Training Ts (%g) and evaluation Ts (%g) do not match.', ...
        Ts, evaluation_source.meta.Ts);
end

training_runs = buildRuns(training_source.datasets, keep_factor, ...
    filter_order, filter_window, "training");
evaluation_runs = buildRuns(evaluation_source.datasets, keep_factor, ...
    filter_order, filter_window, "evaluation");

all_training_x = [training_runs.X];
all_training_u = [training_runs.U];
if isempty(all_training_x)
    error('The training split contains no valid EEFig transitions.');
end

% Scale only, without subtracting a mean: the local consequents do not have
% an affine intercept, so scale-only normalization preserves the origin.
scale_x = prctile(abs(all_training_x), 95, 2);
scale_u = prctile(abs(all_training_u), 95, 2);
minimum_scale = 1e-3;
scale_x = max(scale_x, minimum_scale);
scale_u = max(scale_u, minimum_scale);

training_runs = normalizeRuns(training_runs, scale_x, scale_u);
evaluation_runs = normalizeRuns(evaluation_runs, scale_x, scale_u);

all_training_zeta = [[training_runs.Xn]; [training_runs.Un]];

prepared.training_runs = training_runs;
prepared.evaluation_runs = evaluation_runs;
prepared.scale_x = scale_x;
prepared.scale_u = scale_u;
prepared.training_min = min(all_training_zeta, [], 2);
prepared.training_max = max(all_training_zeta, [], 2);
prepared.Ts = Ts;
prepared.keep_factor = keep_factor;
prepared.filter_order = filter_order;
prepared.filter_window = filter_window;
prepared.training_dataset_file = string(training_dataset_file);
prepared.evaluation_dataset_file = string(evaluation_dataset_file);

fprintf('EEFIG training:   %d transitions from %d independent runs.\n', ...
    sum([training_runs.sample_count]), numel(training_runs));
fprintf('EEFIG evaluation: %d transitions from %d held-out runs.\n', ...
    sum([evaluation_runs.sample_count]), numel(evaluation_runs));
fprintf('Scale [vy r vx delta]: [%g %g %g %g]\n', ...
    scale_x(1), scale_x(2), scale_u(1), scale_u(2));
end

function runs = buildRuns(datasets, keep_factor, filter_order, ...
    filter_window, split_name)
required = {'vy','r','vx','delta'};
runs = repmat(struct('X',[],'U',[],'Xnext',[], ...
    'Xn',[],'Un',[],'Xnextn',[],'sample_count',0, ...
    'source_index',0,'path',"",'event',"",'track_layout',""), ...
    numel(datasets), 1);

for run_idx = 1:numel(datasets)
    data = datasets(run_idx).data;
    lengths = zeros(1, numel(required));
    for signal_idx = 1:numel(required)
        name = required{signal_idx};
        if ~isfield(data, name)
            error('%s run %d is missing data.%s.', split_name, run_idx, name);
        end
        lengths(signal_idx) = numel(data.(name));
    end
    if any(lengths ~= lengths(1))
        error('%s run %d has inconsistent signal lengths.', split_name, run_idx);
    end

    ini = datasets(run_idx).ini;
    fin = datasets(run_idx).fin;
    if isempty(ini), ini = 1; end
    if isempty(fin), fin = lengths(1); end
    fin = min(fin, lengths(1));

    if fin <= ini
        error('%s run %d must contain at least two selected samples.', ...
            split_name, run_idx);
    end
    if fin - ini + 1 < filter_window
        error('%s run %d is shorter than the filter window (%d).', ...
            split_name, run_idx, filter_window);
    end

    selected = ini:fin;
    filtered = struct();
    for signal_idx = 1:numel(required)
        name = required{signal_idx};
        values = data.(name);
        filtered_signal = sgolayfilt_custom( ...
            values(selected), filter_order, filter_window);
        filtered.(name) = filtered_signal(:);
    end

    idx = (1:keep_factor:(numel(selected) - 1))';
    next_idx = idx + 1;
    valid = isfinite(filtered.vy(idx)) & isfinite(filtered.vy(next_idx)) & ...
        isfinite(filtered.r(idx)) & isfinite(filtered.r(next_idx)) & ...
        isfinite(filtered.vx(idx)) & isfinite(filtered.delta(idx));
    idx = idx(valid);
    next_idx = next_idx(valid);
    if isempty(idx)
        error('%s run %d contains no finite EEFig transitions.', ...
            split_name, run_idx);
    end

    runs(run_idx).X = [filtered.vy(idx)'; filtered.r(idx)'];
    runs(run_idx).U = [filtered.vx(idx)'; filtered.delta(idx)'];
    runs(run_idx).Xnext = [filtered.vy(next_idx)'; filtered.r(next_idx)'];
    runs(run_idx).sample_count = numel(idx);
    runs(run_idx).source_index = run_idx;
    runs(run_idx).path = string(datasets(run_idx).path);
    runs(run_idx).event = string(datasets(run_idx).event);
    runs(run_idx).track_layout = string(datasets(run_idx).track_layout);
end
end

function runs = normalizeRuns(runs, scale_x, scale_u)
for run_idx = 1:numel(runs)
    runs(run_idx).Xn = runs(run_idx).X ./ scale_x;
    runs(run_idx).Un = runs(run_idx).U ./ scale_u;
    runs(run_idx).Xnextn = runs(run_idx).Xnext ./ scale_x;
end
end

function assertIndependentRuns(training, evaluation)
training_paths = normalizePaths(string({training.path}));
evaluation_paths = normalizePaths(string({evaluation.path}));
overlap = intersect(training_paths, evaluation_paths);
if ~isempty(overlap)
    error('Training/evaluation leakage: path appears in both splits: %s', ...
        strjoin(overlap, ', '));
end
end

function paths = normalizePaths(paths)
paths = replace(strtrim(paths(:)), '\', '/');
paths = lower(paths);
end
