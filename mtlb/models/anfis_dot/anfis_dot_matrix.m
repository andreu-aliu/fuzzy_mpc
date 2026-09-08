function [A, B, C] = anfis_dot_matrix(X_pred, ~, vx, dt)
% The function returns the discrete state matrices for a given state (predicted + vx)
% x_{k+1} = A x_k + B u_k + C
% x: [y vy psi r delta delta_dot]
% u: steering command

if nargin < 4 || isempty(dt), dt=0.02; end

% Load ANFIS model
persistent anfis_dot;
if(isempty(anfis_dot))
    S = load('anfis_dot.mat', 'anfis_dot');
    anfis_dot = S.anfis_dot;
end

% Normalization params
mu = anfis_dot.norm.mu(:)';
sg = anfis_dot.norm.sigma(:)';
xmin = anfis_dot.norm.x_min(:)';
xmax = anfis_dot.norm.x_max(:)';
assert(all(isfinite(mu)), 'mu invalid');
assert(all(isfinite(sg)), 'sigma invalid');
assert(all(abs(sg) > 1e-8), 'sigma too small or zero');

% Initialization
A = zeros(6);
B = zeros(6,1);
C = zeros(6,1);
vy = X_pred(2); psi = X_pred(3);

% ANFIS input vector [vy r vx delta]
X_in = [X_pred(2) X_pred(4) vx X_pred(5)];

% Detect extrapolation and clamp
mask_low  = X_in < xmin;
mask_high = X_in > xmax;

if any(mask_low) || any(mask_high)

    fprintf('\n===== ANFIS EXTRAPOLATION DETECTED =====\n');

    labels = {'vy','r','vx','delta'};

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
X_in = min(max(X_in, anfis_dot.norm.x_min(:)'), anfis_dot.norm.x_max(:)'); % Clamp
Xin_n = (X_in - mu) ./ sg;

% Y kinematics
A(1,1) = 1;
A(1,2) = cos(psi) * dt;
A(1,3) = (vx * cos(psi) - vy * sin(psi)) * dt;
C(1) = dt*(vx*sin(psi)+vy*cos(psi)) - A(1,2)*vy - A(1,3)*psi;

% Vy dynamics (dot model)
[A_vy_n, b_vy_n, ~] = evalfis_mat(anfis_dot.vy.mat, Xin_n);
A_vy = (A_vy_n(:) ./ sg');
b_vy = b_vy_n - sum(A_vy_n(:) .* (mu' ./ sg'));
active=~(mask_low|mask_high);
A_vy_effective=A_vy.*active(:);
vy_dot_at_op=b_vy+A_vy'*X_in(:);

% vy_{k+1} = vy_k + dt * vy_dot
A(2,2)=1+dt*A_vy_effective(1);
A(2,4)=dt*A_vy_effective(2);
A(2,5)=dt*A_vy_effective(4);
C(2)=dt*(vy_dot_at_op-A_vy_effective(1)*X_pred(2) ...
    -A_vy_effective(2)*X_pred(4)-A_vy_effective(4)*X_pred(5));

% Psi kinematics
A(3,4) = dt;
A(3,3) = 1;

% R dynamics (dot model)
[A_r_n, b_r_n, ~] = evalfis_mat(anfis_dot.r.mat, Xin_n);
A_r = (A_r_n(:) ./ sg');
b_r = b_r_n - sum(A_r_n(:) .* (mu' ./ sg'));
A_r_effective=A_r.*active(:);
r_dot_at_op=b_r+A_r'*X_in(:);

% r_{k+1} = r_k + dt * r_dot
A(4,2)=dt*A_r_effective(1);
A(4,4)=1+dt*A_r_effective(2);
A(4,5)=dt*A_r_effective(4);
C(4)=dt*(r_dot_at_op-A_r_effective(1)*X_pred(2) ...
    -A_r_effective(2)*X_pred(4)-A_r_effective(4)*X_pred(5));

% Steering dynamics (fordward-Euler discretization)
wn = 16.0;
zeta = 0.5;

As_c = [0 1;
     -wn^2  -2*zeta*wn];
Bs_c = [0;
      wn^2];

As = eye(2) + As_c * dt;
Bs = Bs_c * dt;

A(5:6,5:6) = As; % Effect of [delta,delta_dot] on [delta,delta_dot]
B(5:6,1) = Bs;   % Effect of st on [delta,delta_dot]

end
