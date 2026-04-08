function [A, B, C] = anfis_delta_matrix(X_pred, U_pred, vx)
% The funciton returns the discrete state matrices for a given state (predicted + vx)
% x_{k+1} = A x_k + B u_k + C
% x: [y vy psi r delta delta_dot]
% u: [st mz]

% Load ANFIS model
persistent anfis_delta;
if(isempty(anfis_delta))
    S = load('anfis_delta.mat', 'anfis_delta');
    anfis_delta = S.anfis_delta;
end

% Normalization params
mu = anfis_delta.norm.mu(:)';
sg = anfis_delta.norm.sigma(:)';
xmin = anfis_delta.norm.x_min(:)';
xmax = anfis_delta.norm.x_max(:)';
assert(all(isfinite(mu)), 'mu invalid');
assert(all(isfinite(sg)), 'sigma invalid');
assert(all(abs(sg) > 1e-8), 'sigma too small or zero');

% Initialization
dt = 0.02;
A = zeros(6);
B = zeros(6,2);
C = zeros(6,1);
y = X_pred(1); vy = X_pred(2); psi = X_pred(3); r = X_pred(4);

% Anfis matrix for the predicted state [vy r vx delta mz]
X_in = [X_pred(2) X_pred(4) vx X_pred(5) U_pred(2)];

% Detect extrapolation and clamp
mask_low  = X_in < xmin;
mask_high = X_in > xmax;

if any(mask_low) || any(mask_high)

    fprintf('\n===== ANFIS EXTRAPOLATION DETECTED =====\n');

    labels = {'vy','r','vx','st','mz'};

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
X_in = min(max(X_in, anfis_delta.norm.x_min(:)'), anfis_delta.norm.x_max(:)'); % Clamp
Xin_n = (X_in - mu) ./ sg;

% Y kinematics
A(1,1) = 1;
A(1,2) = cos(psi) * dt;
A(1,3) = (vx * cos(psi) - vy * sin(psi)) * dt;
C(1) = dt*(vx*sin(psi)+vy*cos(psi) - A(1,2)*vy - A(1,3)*psi); %vx*sin(psi) * dt; 

% Vy dynamics
[A_vy_n, b_vy_n, ~] = evalfis_mat(anfis_delta.vy.mat, Xin_n);

A_vy = (A_vy_n(:) ./ sg');
b_vy = b_vy_n - sum(A_vy_n(:) .* (mu' ./ sg'));

A(2,2) = A_vy(1) +1; % Effect of vy on vy
A(2,4) = A_vy(2); % Effect of r  on vy
A(2,5) = A_vy(4); % Effect of delta on vy
B(2,2) = A_vy(5); % Effect of mz on vy
C(2)   = b_vy + A_vy(3) * vx; % Effect of vx on vy

% Psi kinematics
A(3,4) = dt;
A(3,3) = 1;

% R dynamics
[A_r_n, b_r_n, ~] = evalfis_mat(anfis_delta.r.mat, Xin_n);

A_r = (A_r_n(:) ./ sg');
b_r = b_r_n - sum(A_r_n(:) .* (mu' ./ sg'));

A(4,2) = A_r(1); % Effect of vy on r
A(4,4) = A_r(2) +1; % Effect of r  on r
A(4,5) = A_r(4); % Effect of delta on r
B(4,2) = A_r(5); % Effect of mz on r
C(4)   = b_r + A_r(3) * vx; % Effect of vx on r

% Steering dynamics
wn = 16.0;
zeta = 0.5;

As_c = [0 1;
     -wn^2  -2*zeta*wn];
Bs_c = [0;
      wn^2];

M = [As_c Bs_c;
     zeros(1,2) 0];
Md = expm(M*dt);

As = Md(1:2,1:2);
Bs = Md(1:2,3);

A(5:6,5:6) = As; % Effect of [delta,delta_dot] on [delta,delta_dot]
B(5:6,1) = Bs;   % Effect of st on [delta,delta_dot]

end


