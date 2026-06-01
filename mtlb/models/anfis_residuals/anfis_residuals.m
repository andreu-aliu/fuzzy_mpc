function x_next = anfis_residuals(X, U, dt)
% X: [y vy psi r delta delta_dot]
% U: [vx st Mtv]

C = num2cell(X);
[y, vy, psi, r, delta, delta_dot] = deal(C{:}); %#ok<ASGLU>
C = num2cell(U);
[vx, st, mz] = deal(C{:});

persistent anfis_residuals;
if isempty(anfis_residuals)
    S = load('anfis_residuals.mat', 'anfis_residuals');
    anfis_residuals = S.anfis_residuals;
end

% Base prediction
x_next = ltv_tv(X, U, dt);

% Residual inputs [vy r vx delta mz]
Xin = [X(2) X(4) vx X(5) mz];
Xin = Xin(:).';
Xin = min(max(Xin, anfis_residuals.norm.x_min(:)'), anfis_residuals.norm.x_max(:)');
Xin_n = (Xin - anfis_residuals.norm.mu(:)') ./ anfis_residuals.norm.sigma(:)';

[~, ~, e_vy] = evalfis_mat(anfis_residuals.vy.mat, Xin_n);
[~, ~, e_r]  = evalfis_mat(anfis_residuals.r.mat,  Xin_n);

x_next(2) = x_next(2) + e_vy;
x_next(4) = x_next(4) + e_r;
end
