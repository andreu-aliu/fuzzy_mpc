%% Prepare real training and evaluation datasets from ROS 2 bags
% Run this script once whenever the source bags or dataset split changes.
% Model training and evaluation scripts should load the generated MAT files
% instead of reading rosbags directly.

clear;
clc;

mtlb_dir = fileparts(mfilename('fullpath'));
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
    "/path/to/real/training/rosbag", [], [], "", [], [];
};

data_paths_eval = {
    "/path/to/real/evaluation/rosbag", [], [], "", [], [];
};

%% Dataset summary

dataset_summary = summarize_dataset_paths(data_paths_training, data_paths_eval);
disp(dataset_summary);

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
