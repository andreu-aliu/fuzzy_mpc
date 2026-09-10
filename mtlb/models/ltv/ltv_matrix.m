function [Ad, Bd, Cd] = ltv_matrix(X_pred, ~, vx, dt)
% x_{k+1} = Ad x_k + Bd u_k + Cd
%
% x: [y vy psi r delta delta_dot]
% u: steering command

if nargin < 4 || isempty(dt), dt = 0.02; end
validateattributes(X_pred,{'numeric'},{'vector','numel',6,'finite'});
validateattributes(vx,{'numeric'},{'scalar','real','finite'});
validateattributes(dt,{'numeric'},{'scalar','real','finite','positive'});

% Predicted operating point
psi   = X_pred(3);
vy = X_pred(2);

% Car parameters
m  = 215;
Iz = 188;
lf = 0.765;
lr = 0.765;
Cf = 2.0 * 1.2705 * 10.5507 * 1104.0;
Cr = 2.0 * 1.2705 * 10.5507 * 1281.5;
wn = 16.0;
zeta = 0.5;

% The dynamic bicycle equations are not valid close to standstill, and their
% 1/vx terms make forward Euler unstable at this sample time below about
% 3 m/s. Use a positive low-speed scheduling floor for the lateral dynamics.
% Measured vx is still used by the global-position kinematics below.
min_dynamic_speed = 3.0;
vx_dynamic = max(vx, min_dynamic_speed);

% Standard small-slip bicycle dynamics with axle cornering stiffnesses.
% y_dot is affine-linearized exactly about the predicted [vy,psi].
A = [0, cos(psi), vx*cos(psi)-vy*sin(psi), 0, 0, 0;
     0, -(Cf+Cr)/(m*vx_dynamic), 0, ...
        (Cr*lr-Cf*lf)/(m*vx_dynamic)-vx_dynamic, Cf/m, 0;
     0, 0, 0, 1, 0, 0;
     0, (Cr*lr-Cf*lf)/(Iz*vx_dynamic), 0, ...
        -(Cf*lf^2+Cr*lr^2)/(Iz*vx_dynamic), Cf*lf/Iz, 0;
     0, 0, 0, 0, 0, 1;
     0, 0, 0, 0, -wn*wn, -2*zeta*wn];
B = [0;0;0;0;0;wn*wn];

C = zeros(6,1);
C(1) = vx*sin(psi) + vy*cos(psi) ...
    - A(1,2)*vy - A(1,3)*psi;

% Discretize matrix

Ad = eye(6,6) + A * dt;
Bd = B * dt;
Cd = C * dt;

end
