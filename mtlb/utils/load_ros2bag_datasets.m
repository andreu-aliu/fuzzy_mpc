function [datasets, meta] = load_ros2bag_datasets(data_paths, Ts, save_name)
% Loads rosbag datasets and saves them in mtlb/data/<save_name>.mat
% data_paths format: {bagPath, ini, fin, event, laps, track_id; ...}
% Supported events: autox, skidpad, trackdrive, acceleration.
% An empty laps value uses the event default: 10 for trackdrive, 1 otherwise.

if nargin < 3 || isempty(save_name)
    error('save_name is required');
end

if nargin < 2 || isempty(Ts)
    error('Ts is required');
end

mtlb_dir = fileparts(fileparts(mfilename('fullpath')));
data_dir = fullfile(mtlb_dir, 'data');
if ~isfolder(data_dir)
    mkdir(data_dir);
end
save_path = fullfile(data_dir, save_name + ".mat");

if size(data_paths, 2) < 3 || size(data_paths, 2) > 6
    error('data_paths must contain between 3 and 6 columns.');
end

datasets = repmat(struct( ...
    'path', "", ...
    'ini', [], ...
    'fin', [], ...
    'event', "unspecified", ...
    'laps', NaN, ...
    'track_id', NaN, ...
    'track_layout', "unspecified", ...
    'data', struct()), size(data_paths, 1), 1);

for i = 1:size(data_paths,1)
    bagPath = data_paths{i,1};
    ini = data_paths{i,2};
    fin = data_paths{i,3};

    if size(data_paths, 2) >= 4 && ~isempty(data_paths{i,4})
        event = lower(string(data_paths{i,4}));
    else
        event = "unspecified";
    end

    if size(data_paths, 2) >= 5
        laps = data_paths{i,5};
    else
        laps = [];
    end

    laps = resolve_laps(event, laps);

    if size(data_paths, 2) >= 6 && ~isempty(data_paths{i,6})
        track_id = data_paths{i,6};
        validateattributes(track_id, {'numeric'}, ...
            {'scalar', 'real', 'finite', 'integer', 'positive'}, ...
            mfilename, 'track_id');
        track_id = double(track_id);
    else
        track_id = NaN;
    end

    if event == "acceleration" || event == "skidpad"
        track_layout = event;
    elseif isnan(track_id)
        if event ~= "unspecified"
            error('Event "%s" requires a track_id.', event);
        end
        track_layout = "unspecified";
    else
        track_layout = "track_" + string(track_id);
    end

    data = read_ros2bag(bagPath, Ts);

    datasets(i).path = string(bagPath);
    datasets(i).ini = ini;
    datasets(i).fin = fin;
    datasets(i).event = event;
    datasets(i).laps = laps;
    datasets(i).track_id = track_id;
    datasets(i).track_layout = track_layout;
    datasets(i).data = data;

    fprintf(['Rosbag %d loaded: %s | %s | %s | %g lap(s) | ' ...
             '%d points\n'], ...
            i, bagPath, event, track_layout, laps, numel(data.vx));
end

meta.Ts = Ts;
meta.saved_at = datetime('now');
meta.data_paths = data_paths;

save(save_path, 'datasets', 'meta', '-v7.3');
fprintf('Saved datasets to %s\n', save_path);
end

function laps = resolve_laps(event, laps_override)
valid_events = ["autox", "skidpad", "trackdrive", "acceleration"];

if event == "unspecified"
    default_laps = NaN;
elseif ~ismember(event, valid_events)
    error('Unsupported event type "%s".', event);
elseif event == "trackdrive"
    default_laps = 10;
else
    default_laps = 1;
end

if isempty(laps_override)
    laps = default_laps;
else
    validateattributes(laps_override, {'numeric'}, ...
        {'scalar', 'real', 'finite', 'integer', 'positive'}, ...
        mfilename, 'laps override');
    laps = double(laps_override);
end
end
