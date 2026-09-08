% Train ANFIS model to replicat system dynamics (dot version)
% inputs: [vy r vx delta] normalized
% output: [vy_dot r_dot]
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
    data.ay = sgolayfilt_custom(data.ay, 3, 21);

    % Resample (keep only some data)
    idx = ini:keep_factor:fin-1;
    idx_next = idx + 1;

    % Concatenate data from all runs
    in.vy = [in.vy; data.vy(idx)];
    in.r  = [in.r ; data.r(idx)];
    in.vx = [in.vx; data.vx(idx)];
    in.delta = [in.delta; data.delta(idx)];
    
    % Predicted is vy_dot, r_dot
    out.vy = [out.vy; data.ay(idx) - data.vx(idx).*data.r(idx)];
    out.r  = [out.r ; (data.r(idx_next)-data.r(idx))./Ts];

    fprintf('Rosbag %d read: %d points.\n', i, nPoints);
end

% Normalization
X = [in.vy in.r in.vx in.delta];
Y = [out.vy out.r];
[Xn, mu, sigma] = zscore(X);
anfis_dot.norm.mu = mu;
anfis_dot.norm.sigma = sigma;
anfis_dot.norm.x_min = min(X,[],1);
anfis_dot.norm.x_max = max(X,[],1);
anfis_dot.Ts = Ts;

% Save max/min of the predictions
anfis_dot.vy.min = min(out.vy);
anfis_dot.vy.max = max(out.vy);
anfis_dot.r.min = min(out.r);
anfis_dot.r.max = max(out.r);

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
save('models/anfis_dot/anfis_dot.mat','anfis_dot')

%% VY_DOT model
load anfis_dot.mat anfis_dot
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

% Save model
save('models/anfis_dot/anfis_dot.mat','anfis_dot')

% Training log:    
% - SC: Clusters:0.35, epoch:200, init:0.15, dec:0.9, inc:1.1 -> 0.521566, 10 rules, 67% RMSE inicial


%% R_DOT model
load anfis_dot.mat anfis_dot
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

% Save model
save('models/anfis_dot/anfis_dot.mat','anfis_dot')

% Training log:    
% - SC: Clusters:0.35, epoch:200, init:0.15, dec:0.9, inc:1.1 -> 0.781126, 10 rules, 78% RMSE inicial


%% Extract and save matrixes
load anfis_dot.mat anfis_dot

anfis_dot.vy.mat = extract_fis(anfis_dot.vy.fis);
anfis_dot.r.mat = extract_fis(anfis_dot.r.fis);

save('models/anfis_dot/anfis_dot.mat','anfis_dot')

%% Model insights

% Model to evaluate
fis = anfis_dot.r.fis;
anfis_model_insights(fis, Xn, trainError, valError, ...
    'inputLabels', {'vy','r','vx','delta'}, ...
    'titlePrefix', 'anfis\_dot.r', ...
    'figBase', 10);
