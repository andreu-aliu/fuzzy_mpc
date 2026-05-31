function [datasets, meta] = load_ros2bag_datasets(data_paths, Ts, save_name)
% Loads rosbag datasets and saves them in mtlb/<save_name>.mat
% data_paths format: {bagPath, ini, fin; ...}

if nargin < 3 || isempty(save_name)
    error('save_name is required');
end

if nargin < 2 || isempty(Ts)
    error('Ts is required');
end

mtlb_dir = fileparts(fileparts(mfilename('fullpath')));
save_path = fullfile(mtlb_dir, save_name + ".mat");

datasets = repmat(struct('path',"",'ini',[],'fin',[],'data',struct()), size(data_paths,1), 1);

for i = 1:size(data_paths,1)
    bagPath = data_paths{i,1};
    ini = data_paths{i,2};
    fin = data_paths{i,3};

    data = read_ros2bag(bagPath, Ts);

    datasets(i).path = string(bagPath);
    datasets(i).ini = ini;
    datasets(i).fin = fin;
    datasets(i).data = data;

    fprintf('Rosbag %d loaded: %s (%d points)\n', i, bagPath, numel(data.vx));
end

meta.Ts = Ts;
meta.saved_at = datetime('now');
meta.data_paths = data_paths;

save(save_path, 'datasets', 'meta', '-v7.3');
fprintf('Saved datasets to %s\n', save_path);
end
