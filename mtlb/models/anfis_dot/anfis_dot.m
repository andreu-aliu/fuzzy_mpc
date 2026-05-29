function x_next = anfis_dot(X, U, dt)
% X: [y vy psi r delta delta_dot]
% U: [vx st Mtv]

% Unpack state and inputs
C = num2cell(X);
[y vy psi r delta delta_dot] = deal(C{:});
C = num2cell(U);
[vx st mz] = deal(C{:});

% Load models
persistent anfis_dot;
if(isempty(anfis_dot))
    S = load('anfis_dot.mat', 'anfis_dot');
    anfis_dot = S.anfis_dot;
end

U_pred = [st; mz];
[A, B, C] = anfis_dot_matrix(X, U_pred, vx);

x_next =  A * X + B * U_pred + C; 

end

