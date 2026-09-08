%% Train ANFIS models for direct one-step lateral-state prediction
% Inputs:  [vy, r, vx, delta]
% Outputs: [vy(k+1), r(k+1)]
cd('/home/andreu/ros_ws/src/as/control/fuzzy_mpc/mtlb'); addpath(genpath('/home/andreu/ros_ws/src/as/control/fuzzy_mpc/mtlb'))
clear

training_dataset_file = fullfile("data","datasets_training.mat");
validation_dataset_file = fullfile("data","datasets_evaluation.mat");
model_file = fullfile("models","anfis_direct","anfis_direct.mat");
keep_factor = 5;
filter_order = 3;
filter_window = 21;
max_samples_per_training_run = 1000;

prepared = prepare_anfis_training_data(training_dataset_file, ...
    validation_dataset_file,'direct',keep_factor,filter_order, ...
    filter_window,max_samples_per_training_run);
Xn_train = prepared.Xn_train;
Y_train = prepared.Y_train;
Xn_val = prepared.Xn_val;
Y_val = prepared.Y_val;

anfis_direct = struct();
anfis_direct.Ts = prepared.Ts;
anfis_direct.norm.mu = prepared.mu;
anfis_direct.norm.sigma = prepared.sigma;
anfis_direct.norm.x_min = prepared.x_min;
anfis_direct.norm.x_max = prepared.x_max;
anfis_direct.vy.min = min(Y_train(:,1));
anfis_direct.vy.max = max(Y_train(:,1));
anfis_direct.r.min = min(Y_train(:,2));
anfis_direct.r.max = max(Y_train(:,2));
anfis_direct.training.dataset_file = training_dataset_file;
anfis_direct.training.validation_dataset_file = validation_dataset_file;
anfis_direct.training.keep_factor = keep_factor;
anfis_direct.training.filter_order = filter_order;
anfis_direct.training.filter_window = filter_window;
anfis_direct.training.max_samples_per_run = max_samples_per_training_run;
anfis_direct.training.training_run_id = prepared.training_run_id;
anfis_direct.training.validation_run_id = prepared.validation_run_id;
save(model_file,'anfis_direct')

%% VY model
% Define model
opt = genfisOptions("SubtractiveClustering");
    % GridPartition:
    % opt.NumMembershipFunctions = [2 2 2 2 3];
    % opt.InputMembershipFunctionType = "gaussmf"; % gbellmf gaussmf trimf trapmf dsigmf psigmf pimf

    % SubtractiveClustering:
    opt.ClusterInfluenceRange = 0.35; %0.5

anfis_direct.vy.init_fis = genfis(Xn_train, Y_train(:,1), opt);

% Training options
opt = anfisOptions;
opt.InitialFIS = anfis_direct.vy.init_fis;
opt.ValidationData = [Xn_val Y_val(:,1)];

opt.EpochNumber = 200;
opt.InitialStepSize = 0.15; %0.01
opt.StepSizeDecreaseRate = 0.9; %0.9
opt.StepSizeIncreaseRate = 1.1; %1.1

opt.DisplayErrorValues = true;
opt.DisplayStepSize    = true;
opt.DisplayANFISInformation = true;
opt.DisplayFinalResults = true;

[vy_final_fis, trainError, ~, vy_validation_fis, valError] = ...
    anfis([Xn_train Y_train(:,1)], opt);
anfis_direct.vy.fis=vy_validation_fis;
anfis_direct.vy.final_epoch_fis=vy_final_fis;
anfis_direct.vy.train_error=trainError;
anfis_direct.vy.validation_error=valError;

% Save model
save(model_file,'anfis_direct')

%% R model
% Define model
opt = genfisOptions("SubtractiveClustering");
    % GridPartition:
    % opt.NumMembershipFunctions = [2 2 2 2 3];
    % opt.InputMembershipFunctionType = "gaussmf"; % gbellmf gaussmf trimf trapmf dsigmf psigmf pimf

    % SubtractiveClustering:
    opt.ClusterInfluenceRange = 0.3; %0.5

anfis_direct.r.init_fis = genfis(Xn_train, Y_train(:,2), opt);

% Training options
opt = anfisOptions;
opt.InitialFIS = anfis_direct.r.init_fis;
opt.ValidationData = [Xn_val Y_val(:,2)];

opt.EpochNumber = 200;
opt.InitialStepSize = 0.10; %0.01
opt.StepSizeDecreaseRate = 0.9; %0.9
opt.StepSizeIncreaseRate = 1.1; %1.1

opt.DisplayErrorValues = true;
opt.DisplayStepSize    = true;
opt.DisplayANFISInformation = true;
opt.DisplayFinalResults = true;

[r_final_fis, trainError, ~, r_validation_fis, valError] = ...
    anfis([Xn_train Y_train(:,2)], opt);
anfis_direct.r.fis=r_validation_fis;
anfis_direct.r.final_epoch_fis=r_final_fis;
anfis_direct.r.train_error=trainError;
anfis_direct.r.validation_error=valError;

% Save model
save(model_file,'anfis_direct')

%% Extract and save matrixes
anfis_direct.vy.mat = extract_fis(anfis_direct.vy.fis);
anfis_direct.r.mat = extract_fis(anfis_direct.r.fis);

save(model_file,'anfis_direct')

%% Model insights

% Model to evaluate
fis = anfis_direct.r.fis;
anfis_model_insights(fis, Xn_train, trainError, valError, ...
    'inputLabels', {'vy','r','vx','delta'}, ...
    'titlePrefix', 'anfis\_direct.r', ...
    'figBase', 20);
