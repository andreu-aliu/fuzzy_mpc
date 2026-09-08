% Train ANFIS model to predict the one-step state residual of the LTV model
% inputs: [vy r vx delta] normalized
% outputs: [e_vy e_r] where e = x_meas_{k+1} - x_ltv_{k+1}
cd('/home/andreu/ros_ws/src/as/control/fuzzy_mpc/mtlb'); addpath(genpath('/home/andreu/ros_ws/src/as/control/fuzzy_mpc/mtlb'))
clear

keep_factor = 5;
validation_fraction = 0.2;
seed = 2;
dataset_file = fullfile("data", "datasets_training_simu.mat");

Ts = 0.02;

%% Load prepared training data
load(dataset_file, 'datasets', 'meta'); %#ok<NASGU>

X = [];
Y = [];

for i = 1:numel(datasets)
    data = datasets(i).data;

    ini = datasets(i).ini;
    if isempty(ini)
        ini = 1;
    end
    fin = datasets(i).fin;
    if isempty(fin)
        fin = numel(data.vx);
    end
    fin = min(fin, numel(data.vx));

    % Filter
    data.vy    = sgolayfilt_custom(data.vy, 3, 21);
    data.r     = sgolayfilt_custom(data.r,  3, 21);
    data.delta = sgolayfilt_custom(data.delta, 3, 21);
    data.vx    = sgolayfilt_custom(data.vx, 3, 21);

    [y_local, psi_local] = localFrameFromXY(data.x, data.y, data.r, data.time);
    delta_dot = gradient(data.delta, Ts);

    idx = ini:keep_factor:(fin-1);
    for k = idx(:)'
        xk = [y_local(k); data.vy(k); psi_local(k); data.r(k); data.delta(k); delta_dot(k)];
        uk = [data.vx(k),data.st(k)];
        
        % Prediction by steering-only LTV model
        x_pred = ltv(xk,uk,Ts);
        x_meas_next = [y_local(k+1); data.vy(k+1); psi_local(k+1); data.r(k+1); data.delta(k+1); delta_dot(k+1)];

        e = x_meas_next - x_pred;
        e(3) = wrapAngle(e(3));

        Xin = [xk(2) xk(4) uk(1) xk(5)]; % [vy r vx delta]
        X = [X; Xin]; %#ok<AGROW>
        Y = [Y; e(2) e(4)]; %#ok<AGROW>
    end

    fprintf('Rosbag %d read: %d points.\n', i, fin-ini);
end

% Normalization
[Xn, mu, sigma] = zscore(X);
anfis_residuals.norm.mu = mu;
anfis_residuals.norm.sigma = sigma;
anfis_residuals.norm.x_min = min(X, [], 1);
anfis_residuals.norm.x_max = max(X, [], 1);
anfis_residuals.Ts = Ts;

anfis_residuals.vy.min = min(Y(:,1));
anfis_residuals.vy.max = max(Y(:,1));
anfis_residuals.r.min  = min(Y(:,2));
anfis_residuals.r.max  = max(Y(:,2));

% Split dataset
N = size(Xn,1);
rng(seed);
perm = randperm(N);
N_val = round(validation_fraction * N);
val_idx = perm(1:N_val);
train_idx = perm(N_val+1:end);

Xn_val = Xn(val_idx, :);
Y_val  = Y(val_idx, :);
Xn_train = Xn(train_idx, :);
Y_train  = Y(train_idx, :);

fprintf('Validation: %d points \nTraining: %d points\n', N_val, N-N_val);
save('models/anfis_residuals/anfis_residuals.mat', 'anfis_residuals')

%% e_vy model
load anfis_residuals.mat anfis_residuals
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
save('models/anfis_residuals/anfis_residuals.mat', 'anfis_residuals')

%% e_r model
load anfis_residuals.mat anfis_residuals
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

% Save model
save('models/anfis_residuals/anfis_residuals.mat', 'anfis_residuals')

% Training log:    
% - SC: Clusters:0.35, epoch:200, init:0.15, dec:0.9, inc:1.1 -> 0.144425, 9 rules, 67% RMSE inicial


%% Extract and save matrices

load anfis_residuals.mat anfis_residuals

anfis_residuals.vy.mat = extract_fis(anfis_residuals.vy.fis);
anfis_residuals.r.mat  = extract_fis(anfis_residuals.r.fis);

save('models/anfis_residuals/anfis_residuals.mat', 'anfis_residuals')


%% Model insights

% Model to evaluate
fis = anfis_residuals.r.fis;
anfis_model_insights(fis, Xn, trainError, valError, ...
    'inputLabels', {'vy','r','vx','delta'}, ...
    'titlePrefix', 'anfis\_delta.r', ...
    'figBase', 0);

%% 




%% Local helpers
function [y_local, psi_local] = localFrameFromXY(xg, yg, r, t)
xg = xg(:); yg = yg(:);
if numel(xg) < 2
    y_local = zeros(size(xg));
    psi_local = zeros(size(xg));
    return
end
dx = gradient(xg);
dy = gradient(yg);

psi_traj = unwrap(atan2(dy, dx));
if all(abs(dx) < 1e-6) && all(abs(dy) < 1e-6)
    dt_vec = [diff(t(:)); mean(diff(t(:)))];
    psi_traj = unwrap(cumsum(r(:) .* dt_vec));
end

psi0 = psi_traj(1);
x0 = xg(1); y0 = yg(1);
dX = xg - x0;
dY = yg - y0;

y_local = -dX*sin(psi0) + dY*cos(psi0);
psi_local = unwrap(psi_traj - psi0);
end

function a = wrapAngle(a)
a = mod(a + pi, 2*pi) - pi;
end
