function [A, B, C] = anfis_residuals_matrix(X_pred, U_pred, vx)
% The funciton returns the discrete state matrices for a given state (predicted + vx)
% x_{k+1} = A x_k + B u_k + C
% x: [y vy psi r delta delta_dot]
% u: [st mz]

% Load ANFIS model
persistent anfis_residuals;
if(isempty(anfis_residuals))
    S = load('anfis_residuals.mat', 'anfis_residuals');   % change name if needed
    anfis_residuals = S.anfis_residuals;
end

% Normalization params
mu = anfis_residuals.norm.mu(:)';
sg = anfis_residuals.norm.sigma(:)';
xmin = anfis_residuals.norm.x_min(:)';
xmax = anfis_residuals.norm.x_max(:)';
assert(all(isfinite(mu)), 'mu invalid');
assert(all(isfinite(sg)), 'sigma invalid');
assert(all(abs(sg) > 1e-8), 'sigma too small or zero');

% Initialization
dt = 0.02;
A = zeros(6);
B = zeros(6,2);
C = zeros(6,1);
y = X_pred(1); %#ok<NASGU>
vy = X_pred(2);
psi = X_pred(3);
r = X_pred(4);
delta = X_pred(5);
delta_dot = X_pred(6); %#ok<NASGU>
st_cmd = U_pred(1);
mz = U_pred(2);

% Vehicle parameters
m = 220;
Iz = 188;
lf = 0.765;
lr = 0.765;
Cf = 1.2705 * 10.5507 * 1104.0;
Cr = 1.2705 * 10.5507 * 1281.5;

% Anfis matrix for the predicted state [vy r vx delta mz]
X_in = [vy r vx delta mz];

% Detect extrapolation and clamp
mask_low  = X_in < xmin;
mask_high = X_in > xmax;

if any(mask_low) || any(mask_high)

    fprintf('\n===== ANFIS EXTRAPOLATION DETECTED =====\n');

    labels = {'vy','r','vx','delta','mz'};

    for j = 1:length(X_in)
        if mask_low(j) || mask_high(j)
            fprintf('%s: value = %+8.4f | min = %+8.4f | max = %+8.4f  <-- OUT\n', ...
                labels{j}, X_in(j), xmin(j), xmax(j));
        else
            fprintf('%s: value = %+8.4f | min = %+8.4f | max = %+8.4f\n', ...
                labels{j}, X_in(j), xmin(j), xmax(j));
        end
    end

    fprintf('========================================\n\n');
end
X_in = min(max(X_in, anfis_residuals.norm.x_min(:)'), anfis_residuals.norm.x_max(:)'); % Clamp
Xin_n = (X_in - mu) ./ sg;

% Y kinematics -------------------------
A(1,1) = 1;
A(1,2) = cos(psi) * dt;
A(1,3) = (vx * cos(psi) - vy * sin(psi)) * dt;
C(1) = dt*(vx*sin(psi)+vy*cos(psi) - A(1,2)*vy - A(1,3)*psi);

% Vy dynamics ---------------------------
[A_vy_n, b_vy_n, ~] = evalfis_mat(anfis_residuals.vy.mat, Xin_n);

A_vy = (A_vy_n(:) ./ sg');
b_vy = b_vy_n - sum(A_vy_n(:) .* (mu' ./ sg'));

% Bicycle nominal model
vx_eff = max(vx, 0.5);
c = cos(delta);
s = sin(delta);

vy_dot_nom = -(Cf*c + Cr)/(m*vx_eff) * vy + ...
             (-((lf*Cf*c - lr*Cr)/(m*vx_eff)) + vx_eff) * r + ...
             Cf*c/m * delta;

vy_nom_next = vy + vy_dot_nom * dt;

% Jacobians of bicycle nominal model
A_vy_nom_vy    = 1 + dt * (-(Cf*c + Cr)/(m*vx_eff));
A_vy_nom_r     = dt * (-((lf*Cf*c - lr*Cr)/(m*vx_eff)) + vx_eff);
A_vy_nom_delta = dt * ( ...
                    (Cf*s/(m*vx_eff)) * vy + ...
                    (lf*Cf*s/(m*vx_eff)) * r + ...
                    (Cf/m) * (c - delta*s) );

% Total vy row = nominal + residual
A(2,2) = A_vy_nom_vy + A_vy(1);
A(2,4) = A_vy_nom_r  + A_vy(2);
A(2,5) = A_vy_nom_delta + A_vy(4);
B(2,2) = A_vy(5);

% Affine term from exact operating-point evaluation
vy_res_bar = A_vy(1)*X_in(1) + A_vy(2)*X_in(2) + A_vy(3)*X_in(3) + A_vy(4)*X_in(4) + A_vy(5)*X_in(5) + b_vy;
vy_bar_next = vy_nom_next + vy_res_bar;
C(2) = vy_bar_next - A(2,2)*vy - A(2,4)*r - A(2,5)*delta - B(2,2)*mz;

% Psi kinematics -------------------------
A(3,4) = dt;
A(3,3) = 1;

% R dynamics -----------------------------
[A_r_n, b_r_n, ~] = evalfis_mat(anfis_residuals.r.mat, Xin_n);

A_r = (A_r_n(:) ./ sg');
b_r = b_r_n - sum(A_r_n(:) .* (mu' ./ sg'));

% Bicycle nominal model
r_dot_nom  = -(lf*Cf*c - lr*Cr)/(Iz*vx_eff) * vy + ...
             -(lf*lf*Cf*c + lr*lr*Cr)/(Iz*vx_eff) * r + ...
             lf*Cf*c/Iz * delta + ...
             mz/Iz;

r_nom_next = r + r_dot_nom * dt;

% Jacobians of bicycle nominal model
A_r_nom_vy    = dt * (-(lf*Cf*c - lr*Cr)/(Iz*vx_eff));
A_r_nom_r     = 1 + dt * (-(lf*lf*Cf*c + lr*lr*Cr)/(Iz*vx_eff));
A_r_nom_delta = dt * ( ...
                   (lf*Cf*s/(Iz*vx_eff)) * vy + ...
                   (lf*lf*Cf*s/(Iz*vx_eff)) * r + ...
                   (lf*Cf/Iz) * (c - delta*s) );
B_r_nom_mz    = dt / Iz;

% Total r row = nominal + residual
A(4,2) = A_r_nom_vy + A_r(1);
A(4,4) = A_r_nom_r  + A_r(2);
A(4,5) = A_r_nom_delta + A_r(4);
B(4,2) = B_r_nom_mz + A_r(5);

% Affine term from exact operating-point evaluation
r_res_bar = A_r(1)*X_in(1) + A_r(2)*X_in(2) + A_r(3)*X_in(3) + A_r(4)*X_in(4) + A_r(5)*X_in(5) + b_r;
r_bar_next = r_nom_next + r_res_bar;
C(4) = r_bar_next - A(4,2)*vy - A(4,4)*r - A(4,5)*delta - B(4,2)*mz;

% Steering dynamics -------------------------
wn = 16.0;
zeta = 0.5;

As_c = [0 1;
     -wn^2  -2*zeta*wn];
Bs_c = [0;
      wn^2];

% Forward-Euler discretization (match ltv_matrix.m)
As = eye(2) + As_c * dt;
Bs = Bs_c * dt;

A(5:6,5:6) = As; % Effect of [delta,delta_dot] on [delta,delta_dot]
B(5:6,1) = Bs;   % Effect of st on [delta,delta_dot]

end
