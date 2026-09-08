function x_next = sim_anfis_delta(X, U, vx_next, dt)
% Using ANFIS dleta models
% X = [x, y, psi, vx, vy, r, delta, delta_dot]
% U = steering command
% dt: timestep [s]

% State and inputs
psi = X(3);
vx = X(4);
vy = X(5);
r = X(6);
delta = X(7);
vel_delta = X(8);
delta = min(max(delta, -0.45), 0.45);
delta_cmd = U(1);

% Load models
persistent anfis_delta;
if(isempty(anfis_delta))
    S = load('anfis_delta.mat', 'anfis_delta');
    anfis_delta = S.anfis_delta;
end

% Build ANFIS input vector and clamp to trained inputs
Xin = [vy r vx delta];
mask_low  = Xin < anfis_delta.norm.x_min;
mask_high = Xin > anfis_delta.norm.x_max;
if any(mask_low) || any(mask_high)
    fprintf("Simulator model: input vector outside training range\n");
    Xin = min(max(Xin, anfis_delta.norm.x_min), anfis_delta.norm.x_max);
end
Xin_n = (Xin - anfis_delta.norm.mu) ./ anfis_delta.norm.sigma;

% Evaluate learned dynamics
[~, ~,dvy] = evalfis_mat(anfis_delta.vy.mat, Xin_n);
[~, ~,dr]  = evalfis_mat(anfis_delta.r.mat,  Xin_n);

% The learned outputs are increments over the dataset sampling period.
if isfield(anfis_delta, 'Ts')
    training_dt = anfis_delta.Ts;
else
    training_dt = 0.02; % Backward compatibility with older MAT files.
end
vy_dot = dvy / training_dt;
r_dot  = dr  / training_dt;

% Kinematics
x_dot   = vx*cos(psi) - vy*sin(psi);
y_dot   = vx*sin(psi) + vy*cos(psi);
psi_dot = r;

% Steering dynamics
wn = 16.0;
zeta = 0.5;

delta_dot = vel_delta;
delta_dot_dot = -(wn*wn) * delta -2*(wn*zeta) * delta_dot + wn*wn*delta_cmd;

% Integration (Euler)
Xdot = [x_dot, y_dot, psi_dot, 0 , vy_dot, r_dot, delta_dot, delta_dot_dot];
x_next = X + Xdot*dt;
% x_next(2) = y_next; % Overwrite with Taylor aproximation of lateral movement
x_next(4) = vx_next;
x_next(7) = min(max(x_next(7), -0.45), 0.45);

end
