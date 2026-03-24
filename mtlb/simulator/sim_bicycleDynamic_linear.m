function x_next = sim_bicycleDynamic_linear(X, U, vx_next, dt)
% BICYCLE DYNAMIC MODEL
% X = [x, y, psi, vx, vy, r]
% U = [delta, M_TV]

% Car parameters
m = 220;
Iz = 188;
lf = 0.765;
lr = 0.765;
Cf = 1.2705 * 10.5507 * 1104.0;
Cr = 1.2705 * 10.5507 * 1281.5;
Rw = 0.2032;
rho = 1.225;
SCd = 1.854;

% State and Inputs
C = num2cell(X);
[x, y, psi, vx, vy, r] = deal(C{:});
C = num2cell(U);
[delta, M_TV] = deal(C{:});

% 1. Slip angles
vx_eff = max(vx, 0.25); % If the vx is too slow, prevent division by zero
alpha_f = delta - atan2((vy + lf*r),vx_eff);
alpha_r = -atan2((vy - lr*r),vx_eff);

% 2. Tire forces from linear Pacejka
Fyf = Cf * alpha_f * 2;
Fyr = Cr * alpha_r * 2;

Fxf = m * vx_next/dt /2;
Fxr = m * vx_next/dt /2;

% 3. Equations of motion
vy_dot = 1/m*(Fxf*sin(delta)+Fyf*cos(delta)+Fyr)-vx*r;
r_dot = (lf*(Fxf*sin(delta) + Fyf*cos(delta)) - lr*Fyr) / Iz;

x_dot = vx*cos(psi)-vy*sin(psi);
y_dot = vx*sin(psi)+vy*cos(psi);
psi_dot = r;

% 4. Integration (Euler)
Xdot = [x_dot, y_dot, psi_dot, vx_next, vy_dot, r_dot];
x_next = X + Xdot*dt;
end