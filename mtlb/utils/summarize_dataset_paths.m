function [event_summary, track_summary, mz_summary] = summarize_dataset_paths( ...
    data_paths_training, data_paths_eval, datasets_training, datasets_eval)
%SUMMARIZE_DATASET_PATHS Summarize the configured dataset split.
% The sixth column (track_id) may be empty for acceleration and skidpad,
% because those events define their own layout. Autox and trackdrive require
% a positive integer track ID. When the optional loaded datasets are given,
% the third output reports whether each selected run contains nonzero Mz.

if nargin ~= 2 && nargin ~= 4
    error(['Use two inputs for path summaries or four inputs to also ' ...
        'summarize loaded Mz data.']);
end

validate_path_table(data_paths_training, 'training');
validate_path_table(data_paths_eval, 'evaluation');

n_training = size(data_paths_training, 1);
all_paths = [data_paths_training; data_paths_eval];
n_runs = size(all_paths, 1);

events = lower(string(all_paths(:, 4)));
track_layouts = strings(n_runs, 1);
laps = zeros(n_runs, 1);

valid_events = ["autox", "skidpad", "trackdrive", "acceleration"];

for i = 1:n_runs
    event = events(i);
    if ~ismember(event, valid_events)
        error('Invalid event "%s" at combined row %d.', event, i);
    end

    track_id = all_paths{i, 6};
    if event == "acceleration" || event == "skidpad"
        if ~isempty(track_id)
            validate_track_id(track_id, i);
        end
        track_layouts(i) = event;
    else
        if isempty(track_id)
            error('Event "%s" requires a track_id at combined row %d.', ...
                  event, i);
        end
        validate_track_id(track_id, i);
        track_layouts(i) = "track_" + string(track_id);
    end

    laps_override = all_paths{i, 5};
    if isempty(laps_override)
        if event == "trackdrive"
            laps(i) = 10;
        else
            laps(i) = 1;
        end
    else
        validateattributes(laps_override, {'numeric'}, ...
            {'scalar', 'real', 'finite', 'integer', 'positive'}, ...
            mfilename, sprintf('laps at combined row %d', i));
        laps(i) = double(laps_override);
    end
end

is_training = false(n_runs, 1);
is_training(1:n_training) = true;

event_summary = build_run_summary(events, is_training);
track_summary = build_lap_summary(track_layouts, laps, is_training);

if nargin == 4
    mz_summary = build_mz_summary(data_paths_training, data_paths_eval, ...
        datasets_training, datasets_eval);
else
    mz_summary = table();
end
end

function summary = build_mz_summary(data_paths_training, data_paths_eval, ...
        datasets_training, datasets_eval)
mz_zero_tolerance = 1e-9;

if numel(datasets_training) ~= size(data_paths_training,1)
    error('Loaded training-run count does not match data_paths_training.');
end
if numel(datasets_eval) ~= size(data_paths_eval,1)
    error('Loaded evaluation-run count does not match data_paths_eval.');
end

datasets = [datasets_training(:); datasets_eval(:)];
split = [repmat("training",numel(datasets_training),1); ...
    repmat("evaluation",numel(datasets_eval),1)];
run_in_split = [(1:numel(datasets_training))'; (1:numel(datasets_eval))'];
n_runs = numel(datasets);

event = strings(n_runs,1);
track_layout = strings(n_runs,1);
path = strings(n_runs,1);
has_nonzero_mz = false(n_runs,1);
nonzero_samples = zeros(n_runs,1);
total_samples = zeros(n_runs,1);
nonzero_percent = zeros(n_runs,1);
max_abs_mz = zeros(n_runs,1);

for i = 1:n_runs
    if ~isfield(datasets(i).data,'mz')
        error('%s run %d has no data.mz field.',split(i),run_in_split(i));
    end

    mz = datasets(i).data.mz(:);
    ini = datasets(i).ini;
    fin = datasets(i).fin;
    if isempty(ini), ini = 1; end
    if isempty(fin), fin = numel(mz); end

    validateattributes(ini,{'numeric'}, ...
        {'scalar','real','finite','integer','>=',1},mfilename,'ini');
    validateattributes(fin,{'numeric'}, ...
        {'scalar','real','finite','integer','<=',numel(mz)},mfilename,'fin');
    if fin < ini
        error('%s run %d has fin < ini.',split(i),run_in_split(i));
    end

    selected_mz = mz(ini:fin);
    if any(~isfinite(selected_mz))
        error('%s run %d contains non-finite Mz samples.', ...
            split(i),run_in_split(i));
    end

    is_nonzero = abs(selected_mz) > mz_zero_tolerance;
    event(i) = string(datasets(i).event);
    track_layout(i) = string(datasets(i).track_layout);
    path(i) = string(datasets(i).path);
    has_nonzero_mz(i) = any(is_nonzero);
    nonzero_samples(i) = sum(is_nonzero);
    total_samples(i) = numel(selected_mz);
    nonzero_percent(i) = 100*mean(is_nonzero);
    max_abs_mz(i) = max(abs(selected_mz));
end

summary = table(split,run_in_split,event,track_layout,has_nonzero_mz, ...
    nonzero_samples,total_samples,nonzero_percent,max_abs_mz,path, ...
    'VariableNames',{'Split','Run','Event','TrackLayout','HasNonzeroMz', ...
    'NonzeroMzSamples','TotalSamples','NonzeroPercent','MaxAbsMz','Path'});
end

function summary = build_run_summary(events, is_training)
[~, first_idx, group_idx] = unique(events);
n_groups = numel(first_idx);

training_runs = accumarray(group_idx, double(is_training), [n_groups, 1]);
evaluation_runs = accumarray(group_idx, double(~is_training), [n_groups, 1]);
total_runs = training_runs + evaluation_runs;
training_percent = 100 * training_runs ./ total_runs;

summary = table( ...
    events(first_idx), ...
    training_runs, ...
    evaluation_runs, ...
    total_runs, ...
    training_percent, ...
    'VariableNames', { ...
        'Event', 'TrainingRuns', 'EvaluationRuns', 'TotalRuns', ...
        'TrainingPercent'});

summary = sortrows(summary, 'Event');
end

function summary = build_lap_summary(track_layouts, laps, is_training)
groups = track_layouts;
[~, first_idx, group_idx] = unique(groups);
n_groups = numel(first_idx);

training_laps = accumarray( ...
    group_idx, laps .* double(is_training), [n_groups, 1]);
evaluation_laps = accumarray( ...
    group_idx, laps .* double(~is_training), [n_groups, 1]);
total_laps = training_laps + evaluation_laps;
training_percent = 100 * training_laps ./ total_laps;

summary = table( ...
    groups(first_idx), ...
    training_laps, ...
    evaluation_laps, ...
    total_laps, ...
    training_percent, ...
    'VariableNames', { ...
        'TrackLayout', 'TrainingLaps', 'EvaluationLaps', 'TotalLaps', ...
        'TrainingPercent'});

summary = sortrows(summary, 'TrackLayout');
end

function validate_path_table(data_paths, split_name)
if ~iscell(data_paths) || size(data_paths, 2) ~= 6
    error('%s paths must be a six-column cell array.', split_name);
end
end

function validate_track_id(track_id, row)
validateattributes(track_id, {'numeric'}, ...
    {'scalar', 'real', 'finite', 'integer', 'positive'}, ...
    mfilename, sprintf('track_id at combined row %d', row));
end
