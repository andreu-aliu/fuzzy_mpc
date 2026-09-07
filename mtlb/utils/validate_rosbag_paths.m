function validate_rosbag_paths(data_paths_training, data_paths_eval)
%VALIDATE_ROSBAG_PATHS Validate every configured ROS 2 bag before loading.
% A valid bag path is a directory containing metadata.yaml and at least one
% MCAP or SQLite3 storage file.

splits = {data_paths_training, data_paths_eval};
split_names = ["training", "evaluation"];
problems = strings(0, 1);
n_paths = 0;

for split_idx = 1:numel(splits)
    data_paths = splits{split_idx};

    if ~iscell(data_paths) || isempty(data_paths) || size(data_paths, 2) < 1
        problems(end + 1) = split_names(split_idx) + ...
            " dataset configuration is empty or invalid"; %#ok<AGROW>
        continue;
    end

    for row = 1:size(data_paths, 1)
        n_paths = n_paths + 1;
        bag_path = string(data_paths{row, 1});
        row_name = sprintf('%s row %d', split_names(split_idx), row);

        if ~isscalar(bag_path) || ismissing(bag_path) || strlength(bag_path) == 0
            problems(end + 1) = row_name + ": path is empty"; %#ok<AGROW>
            continue;
        end

        if ~isfolder(bag_path)
            problems(end + 1) = row_name + ": directory does not exist: " + ...
                bag_path; %#ok<AGROW>
            continue;
        end

        if ~isfile(fullfile(bag_path, 'metadata.yaml'))
            problems(end + 1) = row_name + ...
                ": metadata.yaml was not found in: " + bag_path; %#ok<AGROW>
        end

        mcap_files = dir(fullfile(bag_path, '*.mcap'));
        db3_files = dir(fullfile(bag_path, '*.db3'));
        if isempty(mcap_files) && isempty(db3_files)
            problems(end + 1) = row_name + ...
                ": no .mcap or .db3 storage file was found in: " + ...
                bag_path; %#ok<AGROW>
        end
    end
end

if ~isempty(problems)
    problem_list = "  - " + strjoin(problems, newline + "  - ");
    error('Rosbag path validation failed before import:\n%s', problem_list);
end

fprintf('Validated %d rosbag paths.\n', n_paths);
end
