function x_next = sim_anfis_direct(X, U, vx_next, dt)
% Using ANFIS direct models
% X = [x, y, psi, vx, vy, r]
% U = [delta, M_TV]
% dt: timestep [s]

% State and Inputs
C = num2cell(X);
[x, y, psi, vx, vy, r] = deal(C{:});
C = num2cell(U);
[delta, mz] = deal(C{:});

% Load models
persistent direct_anfis;
if(isempty(direct_anfis))
    S = load('direct_anfis.mat', 'direct_anfis');
    direct_anfis = S.direct_anfis;
end

% Build ANFIS input vector
Xin = [vy r vx delta mz];
Xin_n = (Xin - direct_anfis.norm.mu) ./ direct_anfis.norm.sigma;

% Evaluate learned dynamics
[~, ~,dvy] = evalfis_mat(direct_anfis.vy.mat, Xin_n);
[~, ~,dr]  = evalfis_mat(direct_anfis.r.mat,  Xin_n);

% Convert to derivatives
vy_dot = dvy / dt;
r_dot  = dr  / dt;

% Kinematics
x_dot   = vx*cos(psi) - vy*sin(psi);
y_dot   = vx*sin(psi) + vy*cos(psi);
psi_dot = r;

% Integration (Euler)
Xdot = [x_dot, y_dot, psi_dot, 0, vy_dot, r_dot];
x_next = X + Xdot*dt;
x_next(4) = vx_next;
end
