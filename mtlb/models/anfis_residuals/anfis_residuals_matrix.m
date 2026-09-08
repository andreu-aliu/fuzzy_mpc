function [A, B, C] = anfis_residuals_matrix(X_pred, U_pred, vx, dt)
% x_{k+1} = A x_k + B u_k + C
% x: [y vy psi r delta delta_dot]
% u: steering command

if nargin < 4 || isempty(dt),dt=0.02;end

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

% Base steering-only LTV model.
[A,B,C] = ltv_matrix(X_pred,U_pred,vx,dt);

% Residual inputs [vy r vx delta]
Xin = [X_pred(2) X_pred(4) vx X_pred(5)];
Xin = Xin(:).';
xmin = xmin(:).';
xmax = xmax(:).';
mu = mu(:).';
sg = sg(:).';
Xin = min(max(Xin, xmin), xmax);
Xin_n = (Xin - mu) ./ sg;
mask_low=[X_pred(2) X_pred(4) vx X_pred(5)]<xmin;
mask_high=[X_pred(2) X_pred(4) vx X_pred(5)]>xmax;
active=~(mask_low|mask_high);
if isfield(anfis_residuals,'Ts'),training_dt=anfis_residuals.Ts;else,training_dt=0.02;end
step_scale=dt/training_dt;

% e_vy residual
[A_vy_n, b_vy_n, ~] = evalfis_mat(anfis_residuals.vy.mat, Xin_n);
A_vy = (A_vy_n(:) ./ sg');
b_vy = b_vy_n - sum(A_vy_n(:) .* (mu' ./ sg'));
A_vy_effective=A_vy.*active(:);
e_vy_at_op=b_vy+A_vy'*Xin(:);
A(2,2)=A(2,2)+step_scale*A_vy_effective(1);
A(2,4)=A(2,4)+step_scale*A_vy_effective(2);
A(2,5)=A(2,5)+step_scale*A_vy_effective(4);
C(2)=C(2)+step_scale*(e_vy_at_op-A_vy_effective(1)*X_pred(2) ...
    -A_vy_effective(2)*X_pred(4)-A_vy_effective(4)*X_pred(5));

% e_r residual
[A_r_n, b_r_n, ~] = evalfis_mat(anfis_residuals.r.mat, Xin_n);
A_r = (A_r_n(:) ./ sg');
b_r = b_r_n - sum(A_r_n(:) .* (mu' ./ sg'));
A_r_effective=A_r.*active(:);
e_r_at_op=b_r+A_r'*Xin(:);
A(4,2)=A(4,2)+step_scale*A_r_effective(1);
A(4,4)=A(4,4)+step_scale*A_r_effective(2);
A(4,5)=A(4,5)+step_scale*A_r_effective(4);
C(4)=C(4)+step_scale*(e_r_at_op-A_r_effective(1)*X_pred(2) ...
    -A_r_effective(2)*X_pred(4)-A_r_effective(4)*X_pred(5));
end
