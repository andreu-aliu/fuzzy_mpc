% The function returns discrete next-state matrices for a given predicted state/input
% x_{k+1} = Ad x_k + Bd u_k + Cd
%
% x: [y vy psi r delta delta_dot]
% u: [st mz]

function [Ad, Bd, Cd] = ltv_tv_matrix(X_pred, U_pred, vx)

dt = 0.02;

% Predicted operating point
psi   = X_pred(3);
delta = X_pred(5);

% Car parameters
m  = 220;
Iz = 188;
lf = 0.765;
lr = 0.765;
Cf = 1.2705 * 10.5507 * 1104.0;
Cr = 1.2705 * 10.5507 * 1281.5;
wn = 16.0;
zeta = 0.5;

% Avoid division by zero / bad conditioning at very low speed
vx_eff = max(vx, 0.5);

% Continuous-time affine model:
% xdot = Ac*x + Bc*u + Cc

Ac = zeros(6,6);
Bc = zeros(6,2);
Cc = zeros(6,1);

% y kinematics, same style as direct_anfis_matrix
% y_dot = vy*cos(psi) + vx*sin(psi)
Ac(1,2) = cos(psi);
Cc(1)   = vx * sin(psi);

% vy dynamics
Ac(2,2) = -(Cf*cos(delta) + Cr) / (m * vx_eff);
Ac(2,4) = -((lf*Cf*cos(delta) - lr*Cr) / (m * vx_eff)) + vx_eff;
Bc(2,1) =  Cf*cos(delta) / m;
Bc(2,2) =  0;

% psi kinematics
Ac(3,4) = 1;

% r dynamics
Ac(4,2) = -(lf*Cf*cos(delta) - lr*Cr) / (Iz * vx_eff);
Ac(4,4) = -(lf^2*Cf*cos(delta) + lr^2*Cr) / (Iz * vx_eff);
Bc(4,1) =  lf*Cf*cos(delta) / Iz;
Bc(4,2) =  1 / Iz;

% Steering dynamics (fordward-Euler discretization)
Ac(5,6) = 1;
Ac(6,5) = -(wn * wn);
Ac(6,6) = -2*(wn*zeta);
Bc(6,1) = wn*wn;

Ad = eye(6) + Ac * dt;
Bd = Bc * dt;
Cd = Cc * dt;

end
