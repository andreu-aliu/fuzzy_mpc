function validate_rosbag_paths(data_paths_training, data_paths_eval)
%VALIDATE_ROSBAG_PATHS Validate every configured ROS 2 bag before loading.
% A valid bag path is a directory containing metadata.yaml and at least one
% MCAP or SQLite3 storage file.

splits = {data_paths_training, data_paths_eval};
split_names = ["training", "evaluation"];
problems = strings(0, 1);
n_paths = 0;

% Check repetitions before touching the filesystem so configuration errors
% are reported together with missing or malformed bags.
problems = [problems; duplicate_path_problems( ...
    data_paths_training,"training")];
problems = [problems; duplicate_path_problems( ...
    data_paths_eval,"evaluation")];
problems = [problems; cross_split_duplicate_problems( ...
    data_paths_training,data_paths_eval)];

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

function problems = duplicate_path_problems(data_paths, split_name)
problems = strings(0,1);
if ~iscell(data_paths) || isempty(data_paths) || size(data_paths,2) < 1
    return;
end

paths = normalized_paths(data_paths(:,1));
for first_row = 1:numel(paths)
    if strlength(paths(first_row)) == 0
        continue;
    end
    for second_row = first_row+1:numel(paths)
        if paths(first_row) == paths(second_row)
            problems(end+1,1) = sprintf( ...
                '%s rows %d and %d repeat the same path: %s', ...
                split_name,first_row,second_row,paths(first_row)); %#ok<AGROW>
        end
    end
end
end

function problems = cross_split_duplicate_problems(training_paths,eval_paths)
problems = strings(0,1);
if ~iscell(training_paths) || isempty(training_paths) || ...
        ~iscell(eval_paths) || isempty(eval_paths) || ...
        size(training_paths,2) < 1 || size(eval_paths,2) < 1
    return;
end

training = normalized_paths(training_paths(:,1));
evaluation = normalized_paths(eval_paths(:,1));
for training_row = 1:numel(training)
    if strlength(training(training_row)) == 0
        continue;
    end
    for evaluation_row = 1:numel(evaluation)
        if training(training_row) == evaluation(evaluation_row)
            problems(end+1,1) = sprintf([ ...
                'training row %d and evaluation row %d repeat the same ' ...
                'path: %s'],training_row,evaluation_row, ...
                training(training_row)); %#ok<AGROW>
        end
    end
end
end

function paths = normalized_paths(path_cells)
paths = strip(string(path_cells));
paths = regexprep(paths,'[\\/]+$','');
end
