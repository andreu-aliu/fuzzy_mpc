function [Ad, Bd, Cd] = ltv_matrix(X_pred, U_pred, vx)
% x_{k+1} = Ad x_k + Bd u_k + Cd
%
% x: [y vy psi r delta delta_dot]
% u: [st mz]
    
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
vx_eff = max(vx, 1.0);

% Model matrices
A = [0, cos(psi), vx*cos(psi), 0, 0, 0;
     0, -(Cf*cos(delta)+Cr)/(m*vx_eff), 0, -((lf*Cf*cos(delta)-lr*Cr)/(m*vx_eff))+vx, Cf*cos(delta)/m, 0;
     0, 0, 0, 1, 0, 0;
     0, -(lf*Cf*cos(delta)-lr*Cr)/(Iz*vx_eff), 0, -(lf*lf*Cf*cos(delta)+lr*lr*Cr)/(Iz*vx_eff), lf*Cf*cos(delta)/Iz, 0;
     0, 0, 0, 0, 0, 1;
     0, 0, 0, 0, -wn*wn, -2*zeta*wn];
B = [0, 0;
     0, 0;
     0, 0;
     0, 0;
     0, 0;
     wn*wn, 0];

C = zeros(6,1);

% Discretize matrix

Ad = eye(6,6) + A * dt;
Bd = B * dt;
Cd = C * dt;

% M = [A, B, C;
%      zeros(2,6), zeros(2,2), zeros(2,1);
%      zeros(1,6), zeros(1,2), 0];
% 
% expM = expm(M * dt);
% 
% Ad = expM(1:6, 1:6);
% Bd = expM(1:6, 7:8);
% Cd = expM(1:6, 9);
end