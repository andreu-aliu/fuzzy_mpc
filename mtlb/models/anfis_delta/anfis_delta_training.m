%% Train ANFIS models for one-step lateral-state increments
% Inputs:  [vy, r, vx, delta]
% Outputs: [vy(k+1)-vy(k), r(k+1)-r(k)]

clear;
clc;

mtlb_dir = '/home/andreu/ros_ws/src/as/control/fuzzy_mpc/mtlb';
cd(mtlb_dir);
addpath(genpath(mtlb_dir));

%% Configuration

training_dataset_file = fullfile("data", "datasets_training.mat");
validation_dataset_file = fullfile("data", "datasets_evaluation.mat");
model_file = fullfile("models", "anfis_delta", "anfis_delta.mat");

keep_factor = 5;       % Keep one transition out of every keep_factor samples.
filter_order = 3;
filter_window = 21;
max_samples_per_training_run = 1000; % Cap long runs; never repeat short runs.

%% Load and normalize independent training and validation runs

prepared = prepare_anfis_training_data(training_dataset_file, ...
    validation_dataset_file,'delta',keep_factor,filter_order, ...
    filter_window,max_samples_per_training_run);
Xn_train = prepared.Xn_train;
Y_train = prepared.Y_train;
Xn_val = prepared.Xn_val;
Y_val = prepared.Y_val;

anfis_delta = struct();
anfis_delta.Ts = prepared.Ts;
anfis_delta.norm.mu = prepared.mu;
anfis_delta.norm.sigma = prepared.sigma;
anfis_delta.norm.x_min = prepared.x_min;
anfis_delta.norm.x_max = prepared.x_max;
anfis_delta.vy.min = min(Y_train(:,1));
anfis_delta.vy.max = max(Y_train(:,1));
anfis_delta.r.min = min(Y_train(:,2));
anfis_delta.r.max = max(Y_train(:,2));
anfis_delta.training.dataset_file = training_dataset_file;
anfis_delta.training.validation_dataset_file = validation_dataset_file;
anfis_delta.training.keep_factor = keep_factor;
anfis_delta.training.filter_order = filter_order;
anfis_delta.training.filter_window = filter_window;
anfis_delta.training.max_samples_per_run = max_samples_per_training_run;
anfis_delta.training.training_run_id = prepared.training_run_id;
anfis_delta.training.validation_run_id = prepared.validation_run_id;

%% Train vy-increment model

fis_options = genfisOptions("SubtractiveClustering");
fis_options.ClusterInfluenceRange = 0.35;
anfis_delta.vy.init_fis = genfis(Xn_train, Y_train(:,1), fis_options);

train_options = anfisOptions;
train_options.InitialFIS = anfis_delta.vy.init_fis;
train_options.ValidationData = [Xn_val Y_val(:,1)];
train_options.EpochNumber = 200;
train_options.InitialStepSize = 0.15;
train_options.StepSizeDecreaseRate = 0.9;
train_options.StepSizeIncreaseRate = 1.1;
train_options.DisplayErrorValues = true;
train_options.DisplayStepSize = true;
train_options.DisplayANFISInformation = true;
train_options.DisplayFinalResults = true;

[vy_final_fis, vy_train_error, ~, vy_validation_fis, vy_val_error] = ...
    anfis([Xn_train Y_train(:,1)], train_options);

% The checking-data FIS is the epoch with the lowest held-out error.
anfis_delta.vy.fis = vy_validation_fis;
anfis_delta.vy.final_epoch_fis = vy_final_fis;
anfis_delta.vy.train_error = vy_train_error;
anfis_delta.vy.validation_error = vy_val_error;

%% Train yaw-rate-increment model

fis_options = genfisOptions("SubtractiveClustering");
fis_options.ClusterInfluenceRange = 0.30;
anfis_delta.r.init_fis = genfis(Xn_train, Y_train(:,2), fis_options);

train_options = anfisOptions;
train_options.InitialFIS = anfis_delta.r.init_fis;
train_options.ValidationData = [Xn_val Y_val(:,2)];
train_options.EpochNumber = 200;
train_options.InitialStepSize = 0.10;
train_options.StepSizeDecreaseRate = 0.9;
train_options.StepSizeIncreaseRate = 1.1;
train_options.DisplayErrorValues = true;
train_options.DisplayStepSize = true;
train_options.DisplayANFISInformation = true;
train_options.DisplayFinalResults = true;

[r_final_fis, r_train_error, ~, r_validation_fis, r_val_error] = ...
    anfis([Xn_train Y_train(:,2)], train_options);

anfis_delta.r.fis = r_validation_fis;
anfis_delta.r.final_epoch_fis = r_final_fis;
anfis_delta.r.train_error = r_train_error;
anfis_delta.r.validation_error = r_val_error;

%% Extract the local Takagi-Sugeno matrices and save

anfis_delta.vy.mat = extract_fis(anfis_delta.vy.fis);
anfis_delta.r.mat = extract_fis(anfis_delta.r.fis);
save(model_file, 'anfis_delta');

fprintf('Saved held-out-validation ANFIS model to %s\n', model_file);

%% Model insights

anfis_model_insights(anfis_delta.r.fis, Xn_train, ...
    r_train_error, r_val_error, ...
    'inputLabels', {'vy','r','vx','delta'}, ...
    'titlePrefix', 'anfis\_delta.r', ...
    'figBase', 0);
