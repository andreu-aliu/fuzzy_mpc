function [A, B, C, g] = eefig_matrix(X_pred, ~, vx, dt)
%EEFIG_MATRIX Instantiate the six-state local affine EEFig model.
% x_{k+1} = A*x_k + B*u_delta_k + C
% x = [y vy psi r delta delta_dot]

if nargin < 4 || isempty(dt), dt = 0.02; end
validateattributes(X_pred, {'numeric'}, {'vector','numel',6,'finite'});
validateattributes(vx, {'numeric'}, {'scalar','real','finite'});
validateattributes(dt, {'numeric'}, {'scalar','real','finite','positive'});

model = eefig_runtime('get');
scale_x = model.norm.scale_x(:);
scale_u = model.norm.scale_u(:);
training_dt = model.Ts;

x_lat = X_pred([2, 4]);
u_lat = [vx; X_pred(5)];
zeta_n = [x_lat ./ scale_x; u_lat ./ scale_u];
[A_n, B_n, g] = model.learner.instantiateAB(zeta_n);

% Convert the normalized direct-next-state map to physical coordinates.
Dx = diag(scale_x);
Du = diag(scale_u);
A_lat_training = Dx * A_n / Dx;
B_lat_training = Dx * B_n / Du;

% The learned map is discrete at model.Ts. Interpolate its increment when
% the runtime interval differs, as done for the direct ANFIS model.
step_scale = dt / training_dt;
A_lat = eye(2) + step_scale * (A_lat_training - eye(2));
B_lat = step_scale * B_lat_training;

A = zeros(6);
B = zeros(6, 1);
C = zeros(6, 1);

vy = X_pred(2);
psi = X_pred(3);

% Exact planar kinematics, locally affine in vy and psi.
A(1, 1) = 1;
A(1, 2) = cos(psi) * dt;
A(1, 3) = (vx * cos(psi) - vy * sin(psi)) * dt;
C(1) = dt * (vx * sin(psi) + vy * cos(psi)) ...
    - A(1, 2) * vy - A(1, 3) * psi;

% Learned lateral dynamics. vx is known scheduling information and delta
% is the actual steering state, not an MPC decision variable.
A([2, 4], [2, 4]) = A_lat;
A([2, 4], 5) = B_lat(:, 2);
C([2, 4]) = B_lat(:, 1) * vx;

A(3, 3) = 1;
A(3, 4) = dt;

% Shared second-order steering actuator.
wn = 16.0;
damping = 0.5;
As_c = [0, 1; -wn^2, -2*damping*wn];
Bs_c = [0; wn^2];
A(5:6, 5:6) = eye(2) + As_c * dt;
B(5:6) = Bs_c * dt;
end
