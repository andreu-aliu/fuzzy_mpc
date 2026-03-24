function x_next = sim_anfis_direct(X, U, params, dt)
% x: state vector [x, y, psi, vx, vy, r, w_fl, w_fr, w_rl, w_rr]
% u: inputs [delta, T_fl, T_fr, T_rl, T_rr]
% params: struct of vehicle parameters
% dt: timestep [s]

% State and Inputs
C = num2cell(X);
[x, y, psi, vx, vy, r, w_fl, w_fr, w_rl, w_rr] = deal(C{:});
C = num2cell(U);
[delta, T_fl, T_fr, T_rl, T_rr] = deal(C{:});

% Aggregate torque
T = T_fl + T_fr + T_rl + T_rr;

% Load ANFIS model
persistent anfis_direct;
if isempty(anfis_direct)
    S = load('anfis_direct.mat', 'anfis_direct');  % load once
    anfis_direct = S.anfis_direct;
end

% Build ANFIS input vector
Xin = [vx, vy, r, T, delta];

% Normalize
Xin_n = (Xin - anfis_direct.norm.mu) ./ anfis_direct.norm.sigma;

% Evaluate learned dynamics (increments)
dvx = evalfis(anfis_direct.vx.fis, Xin_n);
dvy = evalfis(anfis_direct.vy.fis, Xin_n);
dr  = evalfis(anfis_direct.r.fis,  Xin_n);

% Convert to derivatives
vx_dot = dvx / dt;
vy_dot = dvy / dt;
r_dot  = dr  / dt;

% Kinematics
x_dot   = vx*cos(psi) - vy*sin(psi);
y_dot   = vx*sin(psi) + vy*cos(psi);
psi_dot = r;

% Integration (Euler)
Xdot = [x_dot, y_dot, psi_dot, vx_dot, vy_dot, r_dot, 0, 0, 0, 0];
x_next = X + Xdot*dt;
end
