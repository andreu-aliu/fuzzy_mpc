%% Train ANFIS models for lateral-state derivatives
% Inputs:  [vy, r, vx, delta]
% Outputs: [vy_dot, r_dot]
cd('/home/andreu/ros_ws/src/as/control/fuzzy_mpc/mtlb'); addpath(genpath('/home/andreu/ros_ws/src/as/control/fuzzy_mpc/mtlb'))
clear

training_dataset_file = fullfile("data","datasets_training.mat");
validation_dataset_file = fullfile("data","datasets_evaluation.mat");
model_file = fullfile("models","anfis_dot","anfis_dot.mat");
keep_factor = 5;
filter_order = 3;
filter_window = 21;
max_samples_per_training_run = 1000;

prepared = prepare_anfis_training_data(training_dataset_file, ...
    validation_dataset_file,'dot',keep_factor,filter_order, ...
    filter_window,max_samples_per_training_run);
Xn_train = prepared.Xn_train;
Y_train = prepared.Y_train;
Xn_val = prepared.Xn_val;
Y_val = prepared.Y_val;

anfis_dot = struct();
anfis_dot.Ts = prepared.Ts;
anfis_dot.norm.mu = prepared.mu;
anfis_dot.norm.sigma = prepared.sigma;
anfis_dot.norm.x_min = prepared.x_min;
anfis_dot.norm.x_max = prepared.x_max;
anfis_dot.vy.min = min(Y_train(:,1));
anfis_dot.vy.max = max(Y_train(:,1));
anfis_dot.r.min = min(Y_train(:,2));
anfis_dot.r.max = max(Y_train(:,2));
anfis_dot.training.dataset_file = training_dataset_file;
anfis_dot.training.validation_dataset_file = validation_dataset_file;
anfis_dot.training.keep_factor = keep_factor;
anfis_dot.training.filter_order = filter_order;
anfis_dot.training.filter_window = filter_window;
anfis_dot.training.max_samples_per_run = max_samples_per_training_run;
anfis_dot.training.training_run_id = prepared.training_run_id;
anfis_dot.training.validation_run_id = prepared.validation_run_id;
save(model_file,'anfis_dot')

%% VY_DOT model
% Define model
opt = genfisOptions("SubtractiveClustering");
    % SubtractiveClustering: 
    opt.ClusterInfluenceRange = 0.35; %0.5

anfis_dot.vy.init_fis = genfis(Xn_train, Y_train(:,1), opt);

% Training options
opt = anfisOptions;
opt.InitialFIS = anfis_dot.vy.init_fis;
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
anfis_dot.vy.fis=vy_validation_fis;
anfis_dot.vy.final_epoch_fis=vy_final_fis;
anfis_dot.vy.train_error=trainError;
anfis_dot.vy.validation_error=valError;

% Save model
save(model_file,'anfis_dot')

% Training log:    
% - SC: Clusters:0.35, epoch:200, init:0.15, dec:0.9, inc:1.1 -> 0.521566, 10 rules, 67% RMSE inicial


%% R_DOT model
% Define model
opt = genfisOptions("SubtractiveClustering");
    % SubtractiveClustering: 
    opt.ClusterInfluenceRange = 0.3; %0.5

anfis_dot.r.init_fis = genfis(Xn_train, Y_train(:,2), opt);

% Training options
opt = anfisOptions;
opt.InitialFIS = anfis_dot.r.init_fis;
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
anfis_dot.r.fis=r_validation_fis;
anfis_dot.r.final_epoch_fis=r_final_fis;
anfis_dot.r.train_error=trainError;
anfis_dot.r.validation_error=valError;

% Save model
save(model_file,'anfis_dot')

% Training log:    
% - SC: Clusters:0.35, epoch:200, init:0.15, dec:0.9, inc:1.1 -> 0.781126, 10 rules, 78% RMSE inicial


%% Extract and save matrixes
anfis_dot.vy.mat = extract_fis(anfis_dot.vy.fis);
anfis_dot.r.mat = extract_fis(anfis_dot.r.fis);

save(model_file,'anfis_dot')

%% Model insights

% Model to evaluate
fis = anfis_dot.r.fis;
anfis_model_insights(fis, Xn_train, trainError, valError, ...
    'inputLabels', {'vy','r','vx','delta'}, ...
    'titlePrefix', 'anfis\_dot.r', ...
    'figBase', 10);
