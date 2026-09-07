%% Prepare real training and evaluation datasets from ROS 2 bags
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
% Replace these example rows with the paths and metadata for the real-car
% rosbags. Add or remove rows as required.

data_paths_training = {
    % 5/7/26
    "/media/andreu/200GB Toshiba/TFM Data/2026-07-05/05-07-2026__run_3", [], [], "autox", [], 1;
    % 8/7/26
    "/media/andreu/200GB Toshiba/TFM Data/2026-07-08/2026-07-08__run_2", [], [], "autox", [], 2;
    "/media/andreu/200GB Toshiba/TFM Data/2026-07-08/2026-07-08__run_5", [], [], "autox", [], 2;
    "/media/andreu/200GB Toshiba/TFM Data/2026-07-08/2026-07-08__run_10", [], [], "autox", [], 2;
    % 12/7/26
    "/media/andreu/200GB Toshiba/TFM Data/2026-07-12/2026-07-12__run_10", [], [], "autox", [], 3;
    "/media/andreu/200GB Toshiba/TFM Data/2026-07-12/2026-07-12__run_11", [], [], "trackdrive", [], 3;
    "/media/andreu/200GB Toshiba/TFM Data/2026-07-12/2026-07-12__run_23", [], [], "autox", [], 3;
    "/media/andreu/200GB Toshiba/TFM Data/2026-07-12/2026-07-12__run_24", [], [], "autox", [], 3;
    "/media/andreu/200GB Toshiba/TFM Data/2026-07-12/2026-07-12__run_47", [], [], "trackdrive", 9, 3;
    "/media/andreu/200GB Toshiba/TFM Data/2026-07-12/2026-07-12__run_50", [], [], "acceleration", [], [];
    "/media/andreu/200GB Toshiba/TFM Data/2026-07-12/2026-07-12__run_57", [], [], "acceleration", [], [];
    "/media/andreu/200GB Toshiba/TFM Data/2026-07-12/2026-07-12__run_58", [], [], "skidpad", [], [];
    "/media/andreu/200GB Toshiba/TFM Data/2026-07-12/2026-07-12__run_65", [], [], "skidpad", [], [];
    "/media/andreu/200GB Toshiba/TFM Data/2026-07-12/2026-07-12__run_69", [], [], "skidpad", [], [];
    % 30/7/26
    "/media/andreu/200GB Toshiba/TFM Data/2026-07-30/2026-07-30__run_7", [], [], "acceleration", [], [];
    "/media/andreu/200GB Toshiba/TFM Data/2026-07-30/2026-07-30__run_15", [], [], "acceleration", [], [];
    "/media/andreu/200GB Toshiba/TFM Data/2026-07-30/2026-07-30__run_20", [], [], "acceleration", [], [];
    "/media/andreu/200GB Toshiba/TFM Data/2026-07-30/2026-07-30__run_29", [], [], "acceleration", [], [];
    "/media/andreu/200GB Toshiba/TFM Data/2026-07-30/2026-07-30__run_31", [], [], "acceleration", [], [];
    "/media/andreu/200GB Toshiba/TFM Data/2026-07-30/2026-07-30__run_38", [], [], "acceleration", [], [];
    "/media/andreu/200GB Toshiba/TFM Data/2026-07-30/2026-07-30__run_39", [], [], "acceleration", [], [];


};

data_paths_eval = {
    "/media/andreu/200GB Toshiba/TFM Data/2026-07-30/2026-07-30__run_41", [], [], "trackdrive", [], 4;
    "/media/andreu/200GB Toshiba/TFM Data/2026-07-12/2026-07-12__run_61", [], [], "skidpad", [], [];
    "/media/andreu/200GB Toshiba/TFM Data/2026-07-12/2026-07-12__run_67", [], [], "skidpad", [], [];
    "/media/andreu/200GB Toshiba/TFM Data/2026-07-12/2026-07-12__run_53", [], [], "acceleration", [], [];
    "/media/andreu/200GB Toshiba/TFM Data/2026-07-30/2026-07-30__run_12", [], [], "acceleration", [], [];
    "/media/andreu/200GB Toshiba/TFM Data/2026-07-30/2026-07-30__run_23", [], [], "acceleration", [], [];
    "/media/andreu/200GB Toshiba/TFM Data/2026-07-30/2026-07-30__run_38", [], [], "acceleration", [], [];
    "/media/andreu/200GB Toshiba/TFM Data/2026-07-12/2026-07-12__run_16", [], [], "autox", [], 3;
    "/media/andreu/200GB Toshiba/TFM Data/2026-07-12/2026-07-12__run_42", [], [], "autox", [], 3;
    "/media/andreu/200GB Toshiba/TFM Data/2026-07-05/05-07-2026__run_4", [], [], "autox", [], 1;
    "/media/andreu/200GB Toshiba/TFM Data/2026-07-08/2026-07-08__run_9", [], [], "autox", [], 2;
    "/media/andreu/200GB Toshiba/TFM Data/2026-07-12/2026-07-12__run_31", [], [], "autox", [], 3;
    "/media/andreu/200GB Toshiba/TFM Data/2026-07-12/2026-07-12__run_8", [], [], "autox", [], 3;
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

fprintf('Preparing %d real training rosbags...\n', size(data_paths_training, 1));
load_ros2bag_datasets(data_paths_training, Ts, "datasets_training");

fprintf('\nPreparing %d real evaluation rosbags...\n', size(data_paths_eval, 1));
load_ros2bag_datasets(data_paths_eval, Ts, "datasets_evaluation");

fprintf('\nReal dataset preparation complete.\n');
fprintf('  Training:   %s\n', ...
        fullfile(mtlb_dir, 'data', 'datasets_training.mat'));
fprintf('  Evaluation: %s\n', ...
        fullfile(mtlb_dir, 'data', 'datasets_evaluation.mat'));
