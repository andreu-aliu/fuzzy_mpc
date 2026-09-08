% Train ANFIS model to replicat system dynamics
% inputs: [vy r vx delta mz] normalized
% output: [delta_vy delta_r] 
cd('/home/andreu/ros_ws/src/as/control/fuzzy_mpc/mtlb'); addpath(genpath('/home/andreu/ros_ws/src/as/control/fuzzy_mpc/mtlb'))
clear all

keep_factor = 5; % Keep one of every - samples
validation_fraction = 0.2; % Define the fraction of data for validation
seed = 2;
dataset_file = fullfile("data", "datasets_training.mat");

Ts = 0.02; % Sampling time for the models [s]

%% Load prepared training data
load(dataset_file, 'datasets', 'meta');

in.vy = []; out.vy = [];
in.r  = []; out.r  = [];
in.vx = [];
in.delta = [];
in.mz = [];

for i = 1:numel(datasets)
    data = datasets(i).data;

    ini = datasets(i).ini;
    if isempty(ini)
        ini = 1;
    end
    fin = datasets(i).fin;
    if isempty(fin)
        fin = size(data.vx,1);
    end
    nPoints = fin-ini;

    % Filter data
    data.r  = sgolayfilt_custom(data.r, 3, 21);
    data.vy = sgolayfilt_custom(data.vy, 3, 21);
    data.delta = sgolayfilt_custom(data.delta, 3, 21);
    data.mz = sgolayfilt_custom(data.mz, 3, 21);
    data.vx = sgolayfilt_custom(data.vx, 3, 21);

    % Resample (keep only some data)
    idx = ini:keep_factor:fin-1;
    idx_next = idx + 1;    

    % Concatenate data from all runs
    in.vy = [in.vy; data.vy(idx)];
    in.r  = [in.r ; data.r(idx)];
    in.vx = [in.vx; data.vx(idx)];
    in.delta = [in.delta; data.delta(idx)];
    in.mz = [in.mz; data.mz(idx)];
    
    % Predicted is delta vy,r
    out.vy = [out.vy; data.vy(idx_next)-data.vy(idx)];
    out.r  = [out.r ; data.r(idx_next)-data.r(idx)];

    fprintf('Rosbag %d read: %d points.\n', i, nPoints);
end

% Normalization
X = [in.vy in.r in.vx in.delta in.mz];
Y = [out.vy out.r];
[Xn, mu, sigma] = zscore(X);
anfis_delta.norm.mu = mu;
anfis_delta.norm.sigma = sigma;
anfis_delta.norm.x_min = min(X,[],1);
anfis_delta.norm.x_max = max(X,[],1);

% Save max/min of the predictions
anfis_delta.vy.min = min(out.vy);
anfis_delta.vy.max = max(out.vy);
anfis_delta.r.min = min(out.r);
anfis_delta.r.max = max(out.r);

% Split dataset for validation
N = size(in.vx,1);
rng(seed);
idx = randperm(N); % 1:N;
N_val = round(validation_fraction * N);

val_idx   = idx(1:N_val);
train_idx = idx(N_val+1:end);

Xn_val   = Xn(val_idx, :);
Y_val    = Y(val_idx, :);

Xn_train = Xn(train_idx, :);
Y_train  = Y(train_idx, :);

fprintf('Validation: %d points \nTraining: %d points\n', N_val, N-N_val);
save('models/anfis_delta/anfis_delta.mat','anfis_delta')

%% VY model
load anfis_delta.mat anfis_delta
% Define model
opt = genfisOptions("SubtractiveClustering");
    % GridPartition:
    % opt.NumMembershipFunctions = [2 2 2 2 3];
    % opt.InputMembershipFunctionType = "gaussmf"; % gbellmf gaussmf trimf trapmf dsigmf psigmf pimf

    % SubtractiveClustering: 
    opt.ClusterInfluenceRange = 0.35; %0.5


anfis_delta.vy.init_fis = genfis(Xn_train, Y_train(:,1), opt);

% Training options
opt = anfisOptions;
opt.InitialFIS = anfis_delta.vy.init_fis;
opt.ValidationData = [Xn_val Y_val(:,1)];

opt.EpochNumber = 200;
opt.InitialStepSize = 0.15; %0.01
opt.StepSizeDecreaseRate = 0.9; %0.9
opt.StepSizeIncreaseRate = 1.1; %1.1

opt.DisplayErrorValues = true;
opt.DisplayStepSize    = true;
opt.DisplayANFISInformation = true;
opt.DisplayFinalResults = true;

[anfis_delta.vy.fis, trainError, ~, fis_val, valError] = anfis([Xn_train Y_train(:,1)], opt);

% Save model
save('models/anfis_delta/anfis_delta.mat','anfis_delta')

% Training log:    
% - SC: Clusters:0.50, epoch:200, init:0.01, dec:0.9, inc:1.1 -> 0.0162133, 4 rules, 67% RMSE inicial
% - SC: Clusters:0.50, epoch:200, init:0.10, dec:0.9, inc:1.1 -> 0.0157829, 4 rules, 66% RMSE inicial
% - SC: Clusters:0.35, epoch:200, init:0.10, dec:0.9, inc:1.1 -> 0.014773 , 7 rules, 62% RMSE inicial
% - SC: Clusters:0.35, epoch:300, init:0.10, dec:0.9, inc:1.1 -> 0.0145514, 7 rules, 60% RMSE inicial
% - SC: Clusters:0.35, epoch:200, init:0.15, dec:0.9, inc:1.1 -> 0.0146064, 7 rules, 58% RMSE inicial *
% - SC: Clusters:0.40, epoch:200, init:0.15, dec:0.9, inc:1.1 -> 0.0148359, 6 rules, 58% RMSE inicial


%% R model
load anfis_delta.mat anfis_delta
% Define model
opt = genfisOptions("SubtractiveClustering");
    % GridPartition:
    % opt.NumMembershipFunctions = [2 2 2 2 3];
    % opt.InputMembershipFunctionType = "gaussmf"; % gbellmf gaussmf trimf trapmf dsigmf psigmf pimf

    % SubtractiveClustering: 
    opt.ClusterInfluenceRange = 0.3; %0.5


anfis_delta.r.init_fis = genfis(Xn_train, Y_train(:,2), opt);

% Training options
opt = anfisOptions;
opt.InitialFIS = anfis_delta.r.init_fis;
opt.ValidationData = [Xn_val Y_val(:,2)];

opt.EpochNumber = 200;
opt.InitialStepSize = 0.10; %0.01
opt.StepSizeDecreaseRate = 0.9; %0.9
opt.StepSizeIncreaseRate = 1.1; %1.1

opt.DisplayErrorValues = true;
opt.DisplayStepSize    = true;
opt.DisplayANFISInformation = true;
opt.DisplayFinalResults = true;

[anfis_delta.r.fis, trainError, ~, fis_val, valError] = anfis([Xn_train Y_train(:,2)], opt);

% Save model
save('models/anfis_delta/anfis_delta.mat','anfis_delta')

% Training log:    
% - SC: Clusters:0.35, epoch:200, init:0.13, dec:0.9, inc:1.1 -> 0.0161662, 7 rules, 72% RMSE inicial
% - SC: Clusters:0.35, epoch:200, init:0.01, dec:0.9, inc:1.1 -> 0.0165883, 7 rules, 78% RMSE inicial
% - SC: Clusters:0.50, epoch:200, init:0.01, dec:0.9, inc:1.1 -> 0.0158009, 4 rules, 74% RMSE inicial
% - SC: Clusters:0.50, epoch:200, init:0.10, dec:0.9, inc:1.1 -> 0.0156756, 4 rules, 74% RMSE inicial *


%% Extract and save matrixes

load anfis_delta.mat anfis_delta

anfis_delta.vy.mat = extract_fis(anfis_delta.vy.fis);
anfis_delta.r.mat = extract_fis(anfis_delta.r.fis);

save('models/anfis_delta/anfis_delta.mat','anfis_delta')

%% Model insights

% Model to evaluate
fis = anfis_delta.r.fis;
anfis_model_insights(fis, Xn, trainError, valError, ...
    'inputLabels', {'vy','r','vx','delta','mz'}, ...
    'titlePrefix', 'anfis\_delta.r', ...
    'figBase', 0);
