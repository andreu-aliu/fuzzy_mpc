function x_next = sim_anfis_delta(X, U, vx_next, dt)
% Using ANFIS dleta models
% X = [x, y, psi, vx, vy, r, delta, delta_dot]
% U = [delta_cmd, M_TV]
% dt: timestep [s]

% State and Inputs
C = num2cell(X);
[x, y, psi, vx, vy, r, delta, vel_delta] = deal(C{:});
delta = min(max(delta, -0.45), 0.45);
C = num2cell(U);
[delta_cmd, mz] = deal(C{:});

% Load models
persistent anfis_delta;
if(isempty(anfis_delta))
    S = load('anfis_delta.mat', 'anfis_delta');
    anfis_delta = S.anfis_delta;
end

% Build ANFIS input vector and clamp to trained inputs
Xin = [vy r vx delta mz];
mask_low  = Xin < anfis_delta.norm.x_min;
mask_high = Xin > anfis_delta.norm.x_max;
if any(mask_low) || any(mask_high)
    fprintf("Simulator model: input vector outside training range\n");
    Xin = min(max(Xin, anfis_delta.norm.x_min), anfis_delta.norm.x_max);
    vy = Xin(1);
    r = Xin(2);
end
Xin_n = (Xin - anfis_delta.norm.mu) ./ anfis_delta.norm.sigma;

% Evaluate learned dynamics
[~, ~,dvy] = evalfis_mat(anfis_delta.vy.mat, Xin_n);
[~, ~,dr]  = evalfis_mat(anfis_delta.r.mat,  Xin_n);

% Convert to derivatives
vy_dot = dvy / dt;
r_dot  = dr  / dt;

% Kinematics
x_dot   = vx*cos(psi) - vy*sin(psi);
y_dot   = vx*sin(psi) + vy*cos(psi);
psi_dot = r;

% Linearized y update (same as matrix model)
Ay_vy  = cos(psi) * dt;
Ay_psi = (vx * cos(psi) - vy * sin(psi)) * dt;
Cy     = dt*(vx*sin(psi) + vy*cos(psi) - Ay_vy*vy - Ay_psi*psi);

y_next = y + Ay_vy*vy + Ay_psi*psi + Cy;

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
