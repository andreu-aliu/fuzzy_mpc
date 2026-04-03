% Train ANFIS model to replicat system dynamics
% state:  [y vy psi r] normalized
% inputs: [delta mz]
% output: [vy_dot r_dot] 
cd('/home/andreu/ros_ws/src/as/control/fuzzy_mpc/mtlb'); addpath(genpath('/home/andreu/ros_ws/src/as/control/fuzzy_mpc/mtlb'))
clear all

% List of data path (csv), start, end
data_paths = {"/home/andreu/bcnemotorsport/data/simu/acceleration_3", [], [];
              "/home/andreu/bcnemotorsport/data/simu/acceleration_9", [], [];
              "/home/andreu/bcnemotorsport/data/simu/skidpad_10", [], [];
              "/home/andreu/bcnemotorsport/data/simu/skidpad_13", [], [];
              "/home/andreu/bcnemotorsport/data/simu/skidpad_15", [], [];
              "/home/andreu/bcnemotorsport/data/simu/skidpad_5", [], [];
              % "/home/andreu/bcnemotorsport/data/simu/trackdrive_FSG", [], [];
              "/home/andreu/bcnemotorsport/data/simu/trackdrive_FSI", [], [];
              "/home/andreu/bcnemotorsport/data/simu/trackdrive_FSS", [], [];
              "/home/andreu/bcnemotorsport/data/simu/teleop", [], [];
              };

keep_factor = 5; % Keep one of every - samples
validation_fraction = 0.2; % Define the fraction of data for validation
seed = 2;

%% Load data
in.vy = []; out.vy = [];
in.r  = []; out.r  = [];
in.vx = [];
in.st = [];
in.mz = [];

Ts = 0.02; % Sampling time for the models [s]

for i = 1:size(data_paths,1)
    data = read_ros2bag(data_paths{i,1}, Ts);

    ini = data_paths{i,2};
    if isempty(ini)
        ini = 1;
    end
    fin = data_paths{i,3};
    if isempty(fin)
        fin = size(data.vx,1);
    end
    nPoints = fin-ini;

    % Filter data
    data.r  = sgolayfilt_custom(data.r, 3, 21);
    data.vy = sgolayfilt_custom(data.vy, 3, 21);
    data.st = sgolayfilt_custom(data.st, 3, 21);
    data.mz = sgolayfilt_custom(data.mz, 3, 21);
    data.vx = sgolayfilt_custom(data.vx, 3, 21);

    % Resample (keep only some data)
    idx = ini:keep_factor:fin-1;
    idx_next = idx + 1;    

    % Concatenate data from all runs
    in.vy = [in.vy; data.vy(idx)];
    in.r  = [in.r ; data.r(idx)];
    in.vx = [in.vx; data.vx(idx)];
    in.st = [in.st; data.st(idx)];
    in.mz = [in.mz; data.mz(idx)];

    out.vy = [out.vy; data.vy(idx_next)-data.vy(idx)];
    out.r  = [out.r ; data.r(idx_next)-data.r(idx)];

    fprintf('Rosbag %d read: %d points.\n', i, nPoints);
end

% Normalization
X = [in.vy in.r in.vx in.st in.mz];
Y = [out.vy out.r];
[Xn, mu, sigma] = zscore(X);
direct_anfis.norm.mu = mu;
direct_anfis.norm.sigma = sigma;
direct_anfis.norm.x_min = min(X,[],1);
direct_anfis.norm.x_max = max(X,[],1);

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
save('models/direct_anfis/direct_anfis.mat','direct_anfis')

%% VY model
load direct_anfis.mat direct_anfis
% Define model
opt = genfisOptions("SubtractiveClustering");
    % GridPartition:
    % opt.NumMembershipFunctions = [2 2 2 2 3];
    % opt.InputMembershipFunctionType = "gaussmf"; % gbellmf gaussmf trimf trapmf dsigmf psigmf pimf

    % SubtractiveClustering: 
    opt.ClusterInfluenceRange = 0.35; %0.5


direct_anfis.vy.init_fis = genfis(Xn_train, Y_train(:,1), opt);

% Training options
opt = anfisOptions;
opt.InitialFIS = direct_anfis.vy.init_fis;
opt.ValidationData = [Xn_val Y_val(:,1)];

opt.EpochNumber = 200;
opt.InitialStepSize = 0.15; %0.01
opt.StepSizeDecreaseRate = 0.9; %0.9
opt.StepSizeIncreaseRate = 1.1; %1.1

opt.DisplayErrorValues = true;
opt.DisplayStepSize    = true;
opt.DisplayANFISInformation = true;
opt.DisplayFinalResults = true;

[direct_anfis.vy.fis, trainError, ~, fis_val, valError] = anfis([Xn_train Y_train(:,1)], opt);

% Save model
save('models/direct_anfis/direct_anfis.mat','direct_anfis')

% Training log:    
% - SC: Clusters:0.50, epoch:200, init:0.01, dec:0.9, inc:1.1 -> 0.0162133, 4 rules, 67% RMSE inicial
% - SC: Clusters:0.50, epoch:200, init:0.10, dec:0.9, inc:1.1 -> 0.0157829, 4 rules, 66% RMSE inicial
% - SC: Clusters:0.35, epoch:200, init:0.10, dec:0.9, inc:1.1 -> 0.014773 , 7 rules, 62% RMSE inicial
% - SC: Clusters:0.35, epoch:300, init:0.10, dec:0.9, inc:1.1 -> 0.0145514, 7 rules, 60% RMSE inicial
% - SC: Clusters:0.35, epoch:200, init:0.15, dec:0.9, inc:1.1 -> 0.0146064, 7 rules, 58% RMSE inicial *
% - SC: Clusters:0.40, epoch:200, init:0.15, dec:0.9, inc:1.1 -> 0.0148359, 6 rules, 58% RMSE inicial


%% R model
load direct_anfis.mat direct_anfis
% Define model
opt = genfisOptions("SubtractiveClustering");
    % GridPartition:
    % opt.NumMembershipFunctions = [2 2 2 2 3];
    % opt.InputMembershipFunctionType = "gaussmf"; % gbellmf gaussmf trimf trapmf dsigmf psigmf pimf

    % SubtractiveClustering: 
    opt.ClusterInfluenceRange = 0.5; %0.5


direct_anfis.r.init_fis = genfis(Xn_train, Y_train(:,2), opt);

% Training options
opt = anfisOptions;
opt.InitialFIS = direct_anfis.r.init_fis;
opt.ValidationData = [Xn_val Y_val(:,2)];

opt.EpochNumber = 200;
opt.InitialStepSize = 0.10; %0.01
opt.StepSizeDecreaseRate = 0.9; %0.9
opt.StepSizeIncreaseRate = 1.1; %1.1

opt.DisplayErrorValues = true;
opt.DisplayStepSize    = true;
opt.DisplayANFISInformation = true;
opt.DisplayFinalResults = true;

[direct_anfis.r.fis, trainError, ~, fis_val, valError] = anfis([Xn_train Y_train(:,2)], opt);

% Save model
save('models/direct_anfis/direct_anfis.mat','direct_anfis')

% Training log:    
% - SC: Clusters:0.35, epoch:200, init:0.13, dec:0.9, inc:1.1 -> 0.0161662, 7 rules, 72% RMSE inicial
% - SC: Clusters:0.35, epoch:200, init:0.01, dec:0.9, inc:1.1 -> 0.0165883, 7 rules, 78% RMSE inicial
% - SC: Clusters:0.50, epoch:200, init:0.01, dec:0.9, inc:1.1 -> 0.0158009, 4 rules, 74% RMSE inicial
% - SC: Clusters:0.50, epoch:200, init:0.10, dec:0.9, inc:1.1 -> 0.0156756, 4 rules, 74% RMSE inicial *


%% Extract and save matrixes

load direct_anfis.mat direct_anfis

direct_anfis.vy.mat = extract_fis(direct_anfis.vy.fis);
direct_anfis.r.mat = extract_fis(direct_anfis.r.fis);

save('models/direct_anfis/direct_anfis.mat','direct_anfis')

%% Model insights

% Model to evaluate
fis = direct_anfis.vy.fis;

% Rules info
fprintf("Number of rules: %d", numel(fis.Rules))
showrule(fis)

% Membership functions 
figure(1);
plotmf(fis,'input',1); title('vx membership'); hold on;
histogram(Xn(:,1), 'Normalization', 'pdf', 'FaceAlpha',0.3,'EdgeColor','none'); hold off;
figure(2);
plotmf(fis,'input',2); title('vy membership'); hold on;
histogram(Xn(:,2), 'Normalization', 'pdf', 'FaceAlpha',0.3,'EdgeColor','none'); hold off;
figure(3);
plotmf(fis,'input',3); title('r membership'); hold on;
histogram(Xn(:,3), 'Normalization', 'pdf', 'FaceAlpha',0.3,'EdgeColor','none'); hold off;
figure(4);
plotmf(fis,'input',4); title('T membership'); hold on;
histogram(Xn(:,4), 'Normalization', 'pdf', 'FaceAlpha',0.3,'EdgeColor','none'); hold off;
figure(5);
plotmf(fis,'input',5); title('delta membership'); hold on;
histogram(Xn(:,5), 'Normalization', 'pdf', 'FaceAlpha',0.3,'EdgeColor','none'); hold off;

% Training progression (of last trained fis)
figure(6);
plot(trainError); hold on; 
plot(valError); hold off;
xlabel('Epoch');
ylabel('Training RMSE');
grid on;
title('ANFIS Training Error');
ylim([0, max([trainError; valError])])
legend('Training Error', 'Validation Error')

fprintf('Model trained to %f factor of RMSE\n', trainError(end)/trainError(1))