% Train ANFIS model to replicat system dynamics on residuals of bicycle model
% inputs: [vy r vx delta mz] normalized
% output: [e_vy e_r] 
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
cache_name = "datasets_training";

Ts = 0.02; % Sampling time for the models [s]

%% Load data
load_ros2bag_datasets(data_paths, Ts, cache_name);

%% Prepare data
load(cache_name + ".mat", 'datasets', 'meta');

in.vy = []; out.vy = [];
in.r  = []; out.r  = [];
in.vx = [];
in.st = [];
in.mz = [];

for i = 1:size(data_paths,1)
    data = datasets(i).data;

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
    data.r  = sgolayfilt_custom(data.r, 3, 11);
    data.vy = sgolayfilt_custom(data.vy, 3, 11);
    data.st = sgolayfilt_custom(data.st, 3, 11);
    data.mz = sgolayfilt_custom(data.mz, 3, 11);
    data.vx = sgolayfilt_custom(data.vx, 3, 11);

    % Compute bicycle dynamics for all points
    vm.vy = NaN(length(data.r),1); vm.r = NaN(length(data.r),1);
    for k = 1:length(data.r)
        xk = [data.vy(k), data.r(k), data.vx(k), data.st(k), data.mz(k)];
        vm.vy(k) = bm_vy(xk, Ts);
        vm.r(k)  = bm_r(xk, Ts);
    end 

    % Resample (keep only some data)
    idx = ini:keep_factor:fin-1;
    idx_next = idx + 1;

    % Concatenate data from all runs
    in.vy = [in.vy; data.vy(idx)];
    in.r  = [in.r ; data.r(idx)];
    in.vx = [in.vx; data.vx(idx)];
    in.st = [in.st; data.st(idx)];
    in.mz = [in.mz; data.mz(idx)];
    
    % Predicted is delta vy,r
    out.vy = [out.vy; data.vy(idx_next)-vm.vy(idx)];
    out.r  = [out.r ; data.r(idx_next)-vm.r(idx)];

    fprintf('Rosbag %d read: %d points.\n', i, nPoints);
end

% Normalization
X = [in.vy in.r in.vx in.st in.mz];
Y = [out.vy out.r];
[Xn, mu, sigma] = zscore(X);
anfis_residuals.norm.mu = mu;
anfis_residuals.norm.sigma = sigma;
anfis_residuals.norm.x_min = min(X,[],1);
anfis_residuals.norm.x_max = max(X,[],1);

% Save max/min of the predictions
anfis_residuals.vy.min = min(out.vy);
anfis_residuals.vy.max = max(out.vy);
anfis_residuals.r.min = min(out.r);
anfis_residuals.r.max = max(out.r);

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
save('models/anfis_residuals/anfis_residuals.mat','anfis_residuals')

%% VY model
load anfis_residuals.mat anfis_residuals
% Define model
opt = genfisOptions("SubtractiveClustering");
    % GridPartition:
    % opt.NumMembershipFunctions = [2 2 2 2 3];
    % opt.InputMembershipFunctionType = "gaussmf"; % gbellmf gaussmf trimf trapmf dsigmf psigmf pimf

    % SubtractiveClustering: 
    opt.ClusterInfluenceRange = 0.35; %0.5


anfis_residuals.vy.init_fis = genfis(Xn_train, Y_train(:,1), opt);

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

[anfis_residuals.vy.fis, trainError, ~, fis_val, valError] = anfis([Xn_train Y_train(:,1)], opt);

% Save model
save('models/anfis_residuals/anfis_residuals.mat','anfis_residuals')

% Training log:    
% - SC: Clusters:0.50, epoch:200, init:0.01, dec:0.9, inc:1.1 -> 0.0162133, 4 rules, 67% RMSE inicial


%% R model
load anfis_residuals.mat anfis_residuals
% Define model
opt = genfisOptions("SubtractiveClustering");
    % GridPartition:
    % opt.NumMembershipFunctions = [2 2 2 2 3];
    % opt.InputMembershipFunctionType = "gaussmf"; % gbellmf gaussmf trimf trapmf dsigmf psigmf pimf

    % SubtractiveClustering: 
    opt.ClusterInfluenceRange = 0.5; %0.5


anfis_residuals.r.init_fis = genfis(Xn_train, Y_train(:,2), opt);

% Training options
opt = anfisOptions;
opt.InitialFIS = anfis_residuals.r.init_fis;
opt.ValidationData = [Xn_val Y_val(:,2)];

opt.EpochNumber = 200;
opt.InitialStepSize = 0.10; %0.01
opt.StepSizeDecreaseRate = 0.9; %0.9
opt.StepSizeIncreaseRate = 1.1; %1.1

opt.DisplayErrorValues = true;
opt.DisplayStepSize    = true;
opt.DisplayANFISInformation = true;
opt.DisplayFinalResults = true;

[anfis_residuals.r.fis, trainError, ~, fis_val, valError] = anfis([Xn_train Y_train(:,2)], opt);

% Save model
save('models/anfis_residuals/anfis_residuals.mat','anfis_residuals')

% Training log:    
% - SC: Clusters:0.35, epoch:200, init:0.13, dec:0.9, inc:1.1 -> 0.0161662, 7 rules, 72% RMSE inicial
% - SC: Clusters:0.35, epoch:200, init:0.01, dec:0.9, inc:1.1 -> 0.0165883, 7 rules, 78% RMSE inicial
% - SC: Clusters:0.50, epoch:200, init:0.01, dec:0.9, inc:1.1 -> 0.0158009, 4 rules, 74% RMSE inicial
% - SC: Clusters:0.50, epoch:200, init:0.10, dec:0.9, inc:1.1 -> 0.0156756, 4 rules, 74% RMSE inicial *


%% Extract and save matrixes

load anfis_residuals.mat anfis_residuals

anfis_residuals.vy.mat = extract_fis(anfis_residuals.vy.fis);
anfis_residuals.r.mat = extract_fis(anfis_residuals.r.fis);

save('models/anfis_residuals/anfis_residuals.mat','anfis_residuals')

%% Model insights

% Model to evaluate
fis = anfis_residuals.vy.fis;

% Rules info
fprintf("Number of rules: %d", numel(fis.Rules))
showrule(fis)

% Membership functions 
figure(1);
plotmf(fis,'input',1); title('vy membership'); hold on;
histogram(Xn(:,1), 'Normalization', 'pdf', 'FaceAlpha',0.3,'EdgeColor','none'); hold off;
figure(2);
plotmf(fis,'input',2); title('r membership'); hold on;
histogram(Xn(:,2), 'Normalization', 'pdf', 'FaceAlpha',0.3,'EdgeColor','none'); hold off;
figure(3);
plotmf(fis,'input',3); title('vx membership'); hold on;
histogram(Xn(:,3), 'Normalization', 'pdf', 'FaceAlpha',0.3,'EdgeColor','none'); hold off;
figure(4);
plotmf(fis,'input',4); title('delta membership'); hold on;
histogram(Xn(:,4), 'Normalization', 'pdf', 'FaceAlpha',0.3,'EdgeColor','none'); hold off;
figure(5);
plotmf(fis,'input',5); title('mz membership'); hold on;
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

%% Run models over a run
load anfis_residuals.mat anfis_residuals

% Select run
run_path = "/home/andreu/bcnemotorsport/data/simu/trackdrive_FSG";
data = read_ros2bag(run_path, Ts);

% Optional crop
ini = 1;
fin = length(data.vx);

% Select steering signal
if isfield(data,'delta')
    steer_sig = data.delta;
elseif isfield(data,'st')
    steer_sig = data.st;
else
    error('No steering field found. Expected data.delta or data.st');
end

% Crop
vx_real = data.vx(ini:fin);
vy_real = data.vy(ini:fin);
r_real  = data.r(ini:fin);
delta_real = steer_sig(ini:fin);
mz_real = data.mz(ini:fin);

N = length(vx_real);
t = (0:N-1)' * Ts;

% Preallocate
vy_bm   = NaN(N,1);
r_bm    = NaN(N,1);
vy_corr = NaN(N,1);
r_corr  = NaN(N,1);

dvy_bm   = NaN(N,1);
dr_bm    = NaN(N,1);
dvy_fis  = NaN(N,1);
dr_fis   = NaN(N,1);
dvy_corr = NaN(N,1);
dr_corr  = NaN(N,1);

% Short names
mu   = anfis_residuals.norm.mu(:)';
sg   = anfis_residuals.norm.sigma(:)';
xmin = anfis_residuals.norm.x_min(:)';
xmax = anfis_residuals.norm.x_max(:)';

for k = 1:N-1
    % Current measured state/input
    vy_k = vy_real(k);
    r_k  = r_real(k);
    vx_k = vx_real(k);
    delta_k = delta_real(k);
    mz_k = mz_real(k);

    xk = [vy_k, r_k, vx_k, delta_k, mz_k];

    % ---- Nominal bicycle model (one-step prediction)
    vy_bm(k) = bm_vy(xk, Ts);
    r_bm(k)  = bm_r(xk, Ts);

    dvy_bm(k) = vy_bm(k) - vy_k;
    dr_bm(k)  = r_bm(k)  - r_k;

    % ---- ANFIS residual correction
    xk_clamped = min(max(xk, xmin), xmax);
    xk_n = (xk_clamped - mu) ./ sg;

    % If evalfis_mat returns the output as 3rd output, use this:
    [~, ~, dvy_fis_k] = evalfis_mat(anfis_residuals.vy.mat, xk_n);
    [~, ~, dr_fis_k ] = evalfis_mat(anfis_residuals.r.mat,  xk_n);

    dvy_fis(k) = dvy_fis_k;
    dr_fis(k)  = dr_fis_k;

    % ---- Nominal + residual corrected prediction
    vy_corr(k) = vy_bm(k) + dvy_fis_k;
    r_corr(k)  = r_bm(k)  + dr_fis_k;

    dvy_corr(k) = vy_corr(k) - vy_k;
    dr_corr(k)  = r_corr(k)  - r_k;
end

% Last sample left as NaN because there is no k+1 target
vy_bm(N)   = NaN;
r_bm(N)    = NaN;
vy_corr(N) = NaN;
r_corr(N)  = NaN;

% Ground-truth next-step signals for visual one-step comparison
vy_real_next = [vy_real(2:end); NaN];
r_real_next  = [r_real(2:end); NaN];

% Plot
fig = figure('Name','ANFIS residual model check on run');

ax1 = subplot(3,1,1);
plot(t, vx_real, 'LineWidth', 1.2);
grid on;
ylabel('v_x [m/s]');
title('Recorded longitudinal speed');

ax2 = subplot(3,1,2);
plot(t, r_real_next, 'LineWidth', 1.4); hold on;
plot(t, r_bm, 'LineWidth', 1.2);
plot(t, r_corr, 'LineWidth', 1.2);
grid on;
ylabel('r [rad/s]');
title('Yaw rate: real next sample vs bicycle vs bicycle+ANFIS');
legend('real r(k+1)','bicycle model','anfis-corrected','Location','best');

ax3 = subplot(3,1,3);
plot(t, vy_real_next, 'LineWidth', 1.4); hold on;
plot(t, vy_bm, 'LineWidth', 1.2);
plot(t, vy_corr, 'LineWidth', 1.2);
grid on;
ylabel('v_y [m/s]');
xlabel('time [s]');
title('Lateral velocity: real next sample vs bicycle vs bicycle+ANFIS');
legend('real v_y(k+1)','bicycle model','anfis-corrected','Location','best');

linkaxes([ax1 ax2 ax3],'x');
%% 



%% Bicycle model functions
function vy_next = bm_vy(X, Ts)

    % Vehicle parameters
    m = 220;
    Iz = 188;
    lf = 0.765;
    lr = 0.765;
    Cf = 1.2705 * 10.5507 * 1104.0;
    Cr = 1.2705 * 10.5507 * 1281.5;
    Rw = 0.2032;
    rho = 1.225;
    SCd = 1.854;
    wn = 16.0;
    zeta = 0.5;
    
    % Variables
    vy = X(1);
    r = X(2);
    vx = max(X(3),0.5);
    delta = X(4);
    mz = X(5);

    vy_dot = -(Cf*cos(delta)+Cr)/(m*vx) * vy + ...
             (-((lf*Cf*cos(delta)-lr*Cr)/(m*vx))+vx) * r + ...
             Cf*cos(delta)/m * delta;

    vy_next = vy + vy_dot * Ts;
end

function r_next = bm_r(X, Ts)

    % Vehicle parameters
    m = 220;
    Iz = 188;
    lf = 0.765;
    lr = 0.765;
    Cf = 1.2705 * 10.5507 * 1104.0;
    Cr = 1.2705 * 10.5507 * 1281.5;
    Rw = 0.2032;
    rho = 1.225;
    SCd = 1.854;
    wn = 16.0;
    zeta = 0.5;

    % Variables
    vy = X(1);
    r = X(2);
    vx = max(X(3),0.5);
    delta = X(4);
    mz = X(5);

    r_dot  = -(lf*Cf*cos(delta)-lr*Cr)/(Iz*vx) * vy + ...
             -(lf*lf*Cf*cos(delta)+lr*lr*Cr)/(Iz*vx) * r + ...
             lf*Cf*cos(delta)/Iz * delta + ...
             1/Iz * mz;

    r_next = r + r_dot * Ts;
end
