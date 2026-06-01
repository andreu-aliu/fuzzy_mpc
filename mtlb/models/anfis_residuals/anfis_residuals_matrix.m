function [A, B, C] = anfis_residuals_matrix(X_pred, U_pred, vx)
% x_{k+1} = A x_k + B u_k + C
% x: [y vy psi r delta delta_dot]
% u: [st mz]

persistent anfis_residuals;
if isempty(anfis_residuals)
    S = load('anfis_residuals.mat', 'anfis_residuals');
    anfis_residuals = S.anfis_residuals;
end

mu = anfis_residuals.norm.mu(:)';
sg = anfis_residuals.norm.sigma(:)';
xmin = anfis_residuals.norm.x_min(:)';
xmax = anfis_residuals.norm.x_max(:)';
assert(all(isfinite(mu)), 'mu invalid');
assert(all(isfinite(sg)), 'sigma invalid');
assert(all(abs(sg) > 1e-8), 'sigma too small or zero');

% Base (LTV-TV)
[A, B, C] = ltv_tv_matrix(X_pred, U_pred, vx);

% Residual inputs [vy r vx delta mz]
mz = U_pred(2);
Xin = [X_pred(2) X_pred(4) vx X_pred(5) mz];
Xin = Xin(:).';
xmin = xmin(:).';
xmax = xmax(:).';
mu = mu(:).';
sg = sg(:).';
Xin = min(max(Xin, xmin), xmax);
Xin_n = (Xin - mu) ./ sg;

% e_vy residual
[A_vy_n, b_vy_n, ~] = evalfis_mat(anfis_residuals.vy.mat, Xin_n);
A_vy = (A_vy_n(:) ./ sg');
b_vy = b_vy_n - sum(A_vy_n(:) .* (mu' ./ sg'));

A(2,2) = A(2,2) + A_vy(1);
A(2,4) = A(2,4) + A_vy(2);
A(2,5) = A(2,5) + A_vy(4);
B(2,2) = B(2,2) + A_vy(5);
C(2)   = C(2) + b_vy + A_vy(3) * vx;

% e_r residual
[A_r_n, b_r_n, ~] = evalfis_mat(anfis_residuals.r.mat, Xin_n);
A_r = (A_r_n(:) ./ sg');
b_r = b_r_n - sum(A_r_n(:) .* (mu' ./ sg'));

A(4,2) = A(4,2) + A_r(1);
A(4,4) = A(4,4) + A_r(2);
A(4,5) = A(4,5) + A_r(4);
B(4,2) = B(4,2) + A_r(5);
C(4)   = C(4) + b_r + A_r(3) * vx;
end
