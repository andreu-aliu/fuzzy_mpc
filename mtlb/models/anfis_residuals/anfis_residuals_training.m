%% Train ANFIS models for the LTV one-step lateral-state residual
% Inputs:  [vy, r, vx, delta]
% Outputs: [e_vy, e_r], where e=x_measured(k+1)-x_LTV(k+1)
cd('/home/andreu/ros_ws/src/as/control/fuzzy_mpc/mtlb'); addpath(genpath('/home/andreu/ros_ws/src/as/control/fuzzy_mpc/mtlb'))
clear

keep_factor = 5;
filter_order = 3;
filter_window = 21;
max_samples_per_training_run = 1000;
training_dataset_file = fullfile("data","datasets_training.mat");
validation_dataset_file = fullfile("data","datasets_evaluation.mat");
model_file = fullfile("models","anfis_residuals","anfis_residuals.mat");

prepared = prepare_anfis_training_data(training_dataset_file, ...
    validation_dataset_file,'residual',keep_factor,filter_order, ...
    filter_window,max_samples_per_training_run);
Xn_train = prepared.Xn_train;
Y_train = prepared.Y_train;
Xn_val = prepared.Xn_val;
Y_val = prepared.Y_val;

anfis_residuals = struct();
anfis_residuals.Ts = prepared.Ts;
anfis_residuals.norm.mu = prepared.mu;
anfis_residuals.norm.sigma = prepared.sigma;
anfis_residuals.norm.x_min = prepared.x_min;
anfis_residuals.norm.x_max = prepared.x_max;
anfis_residuals.vy.min = min(Y_train(:,1));
anfis_residuals.vy.max = max(Y_train(:,1));
anfis_residuals.r.min = min(Y_train(:,2));
anfis_residuals.r.max = max(Y_train(:,2));
anfis_residuals.training.dataset_file = training_dataset_file;
anfis_residuals.training.validation_dataset_file = validation_dataset_file;
anfis_residuals.training.keep_factor = keep_factor;
anfis_residuals.training.filter_order = filter_order;
anfis_residuals.training.filter_window = filter_window;
anfis_residuals.training.max_samples_per_run = max_samples_per_training_run;
anfis_residuals.training.training_run_id = prepared.training_run_id;
anfis_residuals.training.validation_run_id = prepared.validation_run_id;
save(model_file,'anfis_residuals')

%% e_vy model
opt = genfisOptions("SubtractiveClustering");
    % GridPartition:
    % opt.NumMembershipFunctions = [2 2 2 2 3];
    % opt.InputMembershipFunctionType = "gaussmf"; % gbellmf gaussmf trimf trapmf dsigmf psigmf pimf

    % SubtractiveClustering: 
    opt.ClusterInfluenceRange = 0.35; %0.5

anfis_residuals.vy.init_fis = genfis(Xn_train,Y_train(:,1),opt);

% Training options
opt = anfisOptions;
opt.InitialFIS = anfis_residuals.vy.init_fis;
opt.ValidationData = [Xn_val Y_val(:,1)];

opt.EpochNumber = 200;
opt.InitialStepSize = 0.15; %0.01
opt.StepSizeDecreaseRate = 0.9; %0.9
opt.StepSizeIncreaseRate = 1.1; %1.1

opt.DisplayErrorValues = true;
opt.DisplayStepSize    = true;
opt.DisplayANFISInformation = true;
opt.DisplayFinalResults = true;

[vy_final_fis,trainError,~,vy_validation_fis,valError] = ...
    anfis([Xn_train Y_train(:,1)],opt);
anfis_residuals.vy.fis=vy_validation_fis;
anfis_residuals.vy.final_epoch_fis=vy_final_fis;
anfis_residuals.vy.train_error=trainError;
anfis_residuals.vy.validation_error=valError;
save(model_file,'anfis_residuals')

%% e_r model
opt = genfisOptions("SubtractiveClustering");
    % GridPartition:
    % opt.NumMembershipFunctions = [2 2 2 2 3];
    % opt.InputMembershipFunctionType = "gaussmf"; % gbellmf gaussmf trimf trapmf dsigmf psigmf pimf

    % SubtractiveClustering: 
    opt.ClusterInfluenceRange = 0.35; %0.5

anfis_residuals.r.init_fis = genfis(Xn_train, Y_train(:,2), opt);

% Training options
opt = anfisOptions;
opt.InitialFIS = anfis_residuals.r.init_fis;
opt.ValidationData = [Xn_val Y_val(:,2)];

opt.EpochNumber = 200;
opt.InitialStepSize = 0.15; %0.01
opt.StepSizeDecreaseRate = 0.9; %0.9
opt.StepSizeIncreaseRate = 1.1; %1.1

opt.DisplayErrorValues = true;
opt.DisplayStepSize    = true;
opt.DisplayANFISInformation = true;
opt.DisplayFinalResults = true;

[r_final_fis,trainError,~,r_validation_fis,valError] = ...
    anfis([Xn_train Y_train(:,2)],opt);
anfis_residuals.r.fis=r_validation_fis;
anfis_residuals.r.final_epoch_fis=r_final_fis;
anfis_residuals.r.train_error=trainError;
anfis_residuals.r.validation_error=valError;

% Save model
save(model_file,'anfis_residuals')

% Training log:    
% - SC: Clusters:0.35, epoch:200, init:0.15, dec:0.9, inc:1.1 -> 0.144425, 9 rules, 67% RMSE inicial


%% Extract and save matrices

anfis_residuals.vy.mat = extract_fis(anfis_residuals.vy.fis);
anfis_residuals.r.mat  = extract_fis(anfis_residuals.r.fis);

save(model_file,'anfis_residuals')


%% Model insights

% Model to evaluate
fis = anfis_residuals.r.fis;
anfis_model_insights(fis, Xn_train, trainError, valError, ...
    'inputLabels', {'vy','r','vx','delta'}, ...
    'titlePrefix', 'anfis\_residuals.r', ...
    'figBase', 0);
