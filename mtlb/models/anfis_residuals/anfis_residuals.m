function x_next = anfis_residuals(X, U, dt)
% X: [y vy psi r delta delta_dot]
% U: [vx steering_command]

validateattributes(U,{'numeric'},{'vector','numel',2,'finite'});
vx=U(1);

persistent anfis_residuals;
if isempty(anfis_residuals)
    S = load('anfis_residuals.mat', 'anfis_residuals');
    anfis_residuals = S.anfis_residuals;
end

% Base prediction
x_next = ltv(X,U,dt);

% Residual inputs [vy r vx delta]
Xin = [X(2) X(4) vx X(5)];
Xin = Xin(:).';
Xin = min(max(Xin, anfis_residuals.norm.x_min(:)'), anfis_residuals.norm.x_max(:)');
Xin_n = (Xin - anfis_residuals.norm.mu(:)') ./ anfis_residuals.norm.sigma(:)';

[~, ~, e_vy] = evalfis_mat(anfis_residuals.vy.mat, Xin_n);
[~, ~, e_r]  = evalfis_mat(anfis_residuals.r.mat,  Xin_n);

if isfield(anfis_residuals,'Ts'),training_dt=anfis_residuals.Ts;else,training_dt=0.02;end
step_scale=dt/training_dt;

x_next(2)=x_next(2)+step_scale*e_vy;
x_next(4)=x_next(4)+step_scale*e_r;
end
