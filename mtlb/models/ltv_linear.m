function x_next = ltv_linear(X, U, dt)
% X: [y vy psi r]
% U: [vx delta Mtv]
    
% Unpack state and inputs
C = num2cell(X);
[y vy psi r] = deal(C{:});
C = num2cell(U);
[vx delta Mtv] = deal(C{:});

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

% Model matrix (y, vy, psi, r)
if vx < 2
    A = zeros(4); 
    B = zeros(4,1);
else
    A = [0, 1, vx, 0; % cos(psi), vx*cos(psi), 0;
         0, -(Cf*cos(delta)+Cr)/(m*vx), 0, -((lf*Cf*cos(delta)-lr*Cr)/(m*vx))+vx;
         0, 0, 0, 1;
         0, -(lf*Cf*cos(delta)-lr*Cr)/(Iz*vx), 0, -(lf*lf*Cf*cos(delta)+lr*lr*Cr)/(Iz*vx)];
    B = [0;
         Cf*cos(delta)/m;
         0;
         lf*Cf*cos(delta)/Iz];
end

% Discretize matrix
M = [A B; zeros(1,5)];
exp_matrix = expm(M*dt);
C = exp_matrix(1:4,1:4);
D = exp_matrix(1:4,5);

% Next state
x_next = C*X + D*delta;

end