function x_next = direct_anfis(X, U, dt)
% X: [y vy psi r]
% U: [vx delta Mtv]
    
% Unpack state and inputs
C = num2cell(X);
[y vy psi r] = deal(C{:});
C = num2cell(U);
[vx st mz] = deal(C{:});

% Load models
persistent direct_anfis;
if(isempty(direct_anfis))
    S = load('direct_anfis.mat', 'direct_anfis');
    direct_anfis = S.direct_anfis;
end

% Build ANFIS input vector and normalize
Xin = [vy r vx st mz];
Xin_n = (Xin - direct_anfis.norm.mu) ./ direct_anfis.norm.sigma;

% Evaluate learned dynamics
dvy = evalfis(direct_anfis.vy.fis, Xin_n);
dr  = evalfis(direct_anfis.r.fis,  Xin_n);

% Convert to derivatives
vy_dot = dvy / dt;
r_dot  = dr  / dt;

% Kinematics
y_dot   = vx*sin(psi) + vy*cos(psi);%cos(psi) * vy + vx*cos(psi) * psi;
psi_dot = r;

% Next state
Xdot = [y_dot; vy_dot; psi_dot; r_dot];
x_next = X + Xdot*dt;

end