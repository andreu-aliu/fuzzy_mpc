%% Prepare simulation training and evaluation datasets from ROS 2 bags
% Run this script once whenever the source bags or dataset split changes.
% Model training and evaluation scripts should load the generated MAT files
% instead of reading rosbags directly.

clear;
clc;

mtlb_dir = '/home/andreu/ros_ws/src/as/control/fuzzy_mpc/mtlb';
cd(mtlb_dir);
addpath(genpath(mtlb_dir));

%% Dataset configuration

Ts = 0.02; % Common sampling time [s]

% Format: {rosbag path, first sample, last sample, event, laps, track_id}
% Use [] to keep the first or last available sample, respectively.
% Set event manually to: "autox", "skidpad", "trackdrive", or
% "acceleration". Leave laps as [] to use the event default (10 for
% trackdrive and 1 for every other event), or set it explicitly to override
% the default. track_id identifies the circuit/layout used by the run. It
% is required for autox and trackdrive, but may be [] for acceleration and
% skidpad because those events are their own layouts.
% Example: {"/path/to/rosbag", [], [], "trackdrive", 5, 2}.
%
% Keep complete rosbags in only one split to prevent information leakage
% between model training and evaluation.
data_paths_training = {
    "/home/andreu/SIMULATIONS/results_3/run_2/rosbag",  [], [], "autox", [], 1;
    "/home/andreu/SIMULATIONS/results_3/run_3/rosbag",  [], [], "autox", [], 1;
    "/home/andreu/SIMULATIONS/results_3/run_4/rosbag",  [], [], "autox", [], 1;
    "/home/andreu/SIMULATIONS/results_3/run_5/rosbag",  [], [], "autox", [], 1;
    "/home/andreu/SIMULATIONS/results_3/run_6/rosbag",  [], [], "autox", [], 1;
    "/home/andreu/SIMULATIONS/results_3/run_9/rosbag",  [], [], "autox", [], 1;
    "/home/andreu/SIMULATIONS/results_3/run_10/rosbag", [], [], "autox", [], 1;
    "/home/andreu/SIMULATIONS/results_3/run_11/rosbag", [], [], "autox", [], 1;
};

data_paths_eval = {
    "/home/andreu/SIMULATIONS/results_3/run_12/rosbag", [], [], "autox", [], 1;
    "/home/andreu/SIMULATIONS/results_3/run_13/rosbag", [], [], "autox", [], 1;
};

%% Validate rosbag paths

validate_rosbag_paths(data_paths_training, data_paths_eval);

%% Dataset summaries

[event_summary, track_summary] = summarize_dataset_paths( ...
    data_paths_training, data_paths_eval);

fprintf('\nRuns per event:\n');
disp(event_summary);

fprintf('Laps per track layout:\n');
disp(track_summary);

%% Generate MAT files

fprintf('Preparing %d simulation training rosbags...\n', ...
        size(data_paths_training, 1));
[datasets_training, ~] = load_ros2bag_datasets( ...
    data_paths_training, Ts, "datasets_training_simu");

fprintf('\nPreparing %d simulation evaluation rosbags...\n', ...
        size(data_paths_eval, 1));
[datasets_eval, ~] = load_ros2bag_datasets( ...
    data_paths_eval, Ts, "datasets_evaluation_simu");

[~, ~, mz_summary] = summarize_dataset_paths(data_paths_training, ...
    data_paths_eval, datasets_training, datasets_eval);
print_mz_summary(mz_summary);

fprintf('\nSimulation dataset preparation complete.\n');
fprintf('  Training:   %s\n', ...
        fullfile(mtlb_dir, 'data', 'datasets_training_simu.mat'));
fprintf('  Evaluation: %s\n', ...
        fullfile(mtlb_dir, 'data', 'datasets_evaluation_simu.mat'));

function print_mz_summary(mz_summary)
nonzero_runs = mz_summary(mz_summary.HasNonzeroMz,:);
fprintf('\nYaw-moment data audit (|Mz| > 1e-9 Nm):\n');
if isempty(nonzero_runs)
    fprintf('All selected Mz samples are zero in every run.\n');
else
    fprintf('%d of %d runs contain nonzero Mz samples:\n', ...
        height(nonzero_runs),height(mz_summary));
    disp(nonzero_runs);
end
end
