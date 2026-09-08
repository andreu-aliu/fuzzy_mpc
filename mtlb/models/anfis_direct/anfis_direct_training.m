% Train ANFIS model to replicat system dynamics
% inputs: [vy r vx delta] normalized
% output: [vy_next r_next]
cd('/home/andreu/ros_ws/src/as/control/fuzzy_mpc/mtlb'); addpath(genpath('/home/andreu/ros_ws/src/as/control/fuzzy_mpc/mtlb'))
clear

keep_factor = 5; % Keep one of every - samples
validation_fraction = 0.2; % Define the fraction of data for validation
seed = 2;
dataset_file = fullfile("data", "datasets_training_simu.mat");

Ts = 0.02; % Sampling time for the models [s]

%% Load prepared training data
load(dataset_file, 'datasets', 'meta');

in.vy = []; out.vy = [];
in.r  = []; out.r  = [];
in.vx = [];
in.delta = [];

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
    data.vx = sgolayfilt_custom(data.vx, 3, 21);

    % Resample (keep only some data)
    idx = ini:keep_factor:fin-1;
    idx_next = idx + 1;

    % Concatenate data from all runs
    in.vy = [in.vy; data.vy(idx)];
    in.r  = [in.r ; data.r(idx)];
    in.vx = [in.vx; data.vx(idx)];
    in.delta = [in.delta; data.delta(idx)];
    
    % Predicted is next vy,r
    out.vy = [out.vy; data.vy(idx_next)];
    out.r  = [out.r ; data.r(idx_next)];

    fprintf('Rosbag %d read: %d points.\n', i, nPoints);
end

% Normalization
X = [in.vy in.r in.vx in.delta];
Y = [out.vy out.r];
[Xn, mu, sigma] = zscore(X);
anfis_direct.norm.mu = mu;
anfis_direct.norm.sigma = sigma;
anfis_direct.norm.x_min = min(X,[],1);
anfis_direct.norm.x_max = max(X,[],1);
anfis_direct.Ts = Ts;

% Save max/min of the predictions
anfis_direct.vy.min = min(out.vy);
anfis_direct.vy.max = max(out.vy);
anfis_direct.r.min = min(out.r);
anfis_direct.r.max = max(out.r);

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
save('models/anfis_direct/anfis_direct.mat','anfis_direct')

%% VY model
load anfis_direct.mat anfis_direct
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

% Save model
save('models/anfis_direct/anfis_direct.mat','anfis_direct')

%% R model
load anfis_direct.mat anfis_direct
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

% Save model
save('models/anfis_direct/anfis_direct.mat','anfis_direct')

%% Extract and save matrixes
load anfis_direct.mat anfis_direct

anfis_direct.vy.mat = extract_fis(anfis_direct.vy.fis);
anfis_direct.r.mat = extract_fis(anfis_direct.r.fis);

save('models/anfis_direct/anfis_direct.mat','anfis_direct')

%% Model insights

% Model to evaluate
fis = anfis_direct.r.fis;
anfis_model_insights(fis, Xn, trainError, valError, ...
    'inputLabels', {'vy','r','vx','delta'}, ...
    'titlePrefix', 'anfis\_direct.r', ...
    'figBase', 20);
