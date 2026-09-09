function tests = test_eefig_learning
%TEST_EEFIG_LEARNING Regression tests for the TS-EEFIG implementation.

tests = functiontests(localfunctions);
end

function setupOnce(testCase)
mtlb_dir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(mtlb_dir, 'utils'));
testCase.TestData.mtlb_dir = mtlb_dir;
end

function testOfflineLinearIdentification(testCase)
rng(7);
A = [0.96, 0.04; -0.03, 0.91];
B = [0.08; 0.05];
N = 400;
X = zeros(2, N);
U = 0.5 * randn(1, N - 1);

for k = 1:N-1
    X(:, k + 1) = A * X(:, k) + B * U(:, k);
end

learner = EEFIGLearning(2, 1, struct('phi', 30, 'min_initial_samples', 30));
learner.trainOffline(X, U);

Y = zeros(2, N - 1);
for k = 1:N-1
    Y(:, k) = learner.predict(X(:, k), U(:, k));
end

rmse = sqrt(mean((Y - X(:, 2:end)).^2, 'all'));
verifyLessThan(testCase, rmse, 1e-10);
verifyGreaterThanOrEqual(testCase, learner.NG, 1);
verifyTrue(testCase, all(isfinite(Y), 'all'));
end

function testAntecedentStatisticsAndBounds(testCase)
Zeta = [-1.0, -0.5, 0.2, 0.8; ...
         0.4, -0.2, 0.1, -0.3; ...
        -0.2,  0.1, 0.3, -0.1];
Xnext = [0.1, 0.2, 0.3, 0.4; -0.1, 0.0, 0.1, 0.2];

learner = EEFIGLearning(2, 1, struct('phi', 4, 'min_initial_samples', 4));
learner.trainOfflinePairs(Zeta, Xnext);
gran = learner.granules{1};

verifySize(testCase, gran.S, [3, 3]);
verifySize(testCase, gran.T, [3, 3]);
verifySize(testCase, gran.s_count, [3, 1]);
verifySize(testCase, gran.t_count, [3, 1]);
verifyGreaterThan(testCase, min(gran.zeta_max - gran.zeta_min), 0);
verifyLessThanOrEqual(testCase, gran.zeta_min, gran.nu);
verifyGreaterThanOrEqual(testCase, gran.zeta_max, gran.nu);
verifyGreaterThan(testCase, min(eig(gran.Sigma)), 0);
verifyGreaterThan(testCase, min(eig(gran.SigmaInv)), 0);
end

function testAuxiliaryGranuleUsesPhiWindow(testCase)
params = struct('phi', 4, 'min_initial_samples', 4);
learner = EEFIGLearning(1, 1, params);
Zeta = [0, 1, 2, 3, 4; 1, 1, 2, 3, 5];

for k = 1:size(Zeta, 2)
    learner.updatePair(Zeta(:, k), Zeta(1, k), 'offline');
end

expected_window = Zeta(:, end-params.phi+1:end);
expected_cov = cov(expected_window');
verifyEqual(testCase, learner.tracker_nu, mean(expected_window, 2), 'AbsTol', 1e-12);
verifyEqual(testCase, pinv(learner.tracker_C), expected_cov, 'AbsTol', 1e-7);
end

function testCreationAndRLSAreMutuallyExclusive(testCase)
params = struct('phi', 3, 'min_initial_samples', 3, ...
    'n_anomaly_max', 0, 'use_c_separation', false);
learner = EEFIGLearning(1, 1, params);

learner.updatePair([0; 0], 0, 'online');
learner.updatePair([0.01; 0], 0.01, 'online');
learner.updatePair([-0.01; 0], -0.01, 'online');
status = learner.updatePair([20; 1], 3, 'online');

verifyTrue(testCase, status.created_new_granule);
new_granule = learner.granules{end};
expected_theta = learner.xNextWindow * pinv(learner.zetaWindow);
verifyEqual(testCase, new_granule.Theta, expected_theta, 'AbsTol', 1e-12);
end

function testPJGAllowsUsefulEvolution(testCase)
rng(12);
learner = EEFIGLearning(1, 1, struct('phi', 20, 'min_initial_samples', 20));
updated = 0;

for k = 1:160
    zeta = 0.1 * randn(2, 1);
    status = learner.updatePair(zeta, 0.7 * zeta(1) - 0.1 * zeta(2), 'offline');
    updated = updated + numel(setdiff(status.accepted_idx, status.reverted_idx));
end

verifyGreaterThan(testCase, updated, 0);
verifyGreaterThan(testCase, learner.granules{1}.n_samples, 20);
verifyGreaterThan(testCase, learner.performanceIndex(1), 0);
end

function testOnlineRLSAdaptsToDynamicsChange(testCase)
rng(9);
A1 = [0.96, 0.03; -0.02, 0.90];
A2 = [0.88, 0.12; -0.08, 0.82];
B = [0.08; 0.04];
N = 300;
X = zeros(2, N);
U = 0.5 * randn(1, N - 1);

for k = 1:N-1
    X(:, k + 1) = A1 * X(:, k) + B * U(:, k);
end

learner = EEFIGLearning(2, 1, struct('phi', 40, 'min_initial_samples', 40));
learner.trainOffline(X, U);

M = 180;
Xchanged = zeros(2, M);
Xchanged(:, 1) = X(:, end);
Uchanged = 0.5 * randn(1, M - 1);
prediction_error = zeros(1, M - 1);

for k = 1:M-1
    Xchanged(:, k + 1) = A2 * Xchanged(:, k) + B * Uchanged(:, k);
    prediction = learner.predict(Xchanged(:, k), Uchanged(:, k));
    prediction_error(k) = norm(prediction - Xchanged(:, k + 1));
    learner.updateOnline(Xchanged(:, k), Uchanged(:, k), Xchanged(:, k + 1));
end

early_rmse = sqrt(mean(prediction_error(1:25).^2));
late_rmse = sqrt(mean(prediction_error(end-24:end).^2));
verifyLessThan(testCase, late_rmse, 0.1 * early_rmse);
end

function testPaperDefaults(testCase)
learner = EEFIGLearning(2, 1, struct());
verifyEqual(testCase, learner.n_anomaly_max, 5);
verifyEqual(testCase, learner.rls_forgetting, 0.99);
verifyEqual(testCase, learner.rls_mode, 'global');
verifyTrue(testCase, learner.use_pjg_quality_check);

gran = TSGranule(2, 1, zeros(3, 1), zeros(2, 1), struct());
verifyEqual(testCase, gran.lambda, 3);
verifyEqual(testCase, gran.iota, 0.3);
verifyEqual(testCase, gran.P_rls, 1e5 * eye(3));
end
