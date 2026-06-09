function x_next = anfis_direct(X, U, dt)
% X: [y vy psi r delta delta_dot]
% U: [vx st Mtv]
    
% Unpack state and inputs
C = num2cell(X);
[y vy psi r delta delta_dot] = deal(C{:});
C = num2cell(U);
[vx st mz] = deal(C{:});

% Load models
persistent anfis_direct;
if(isempty(anfis_direct))
    S = load('anfis_direct.mat', 'anfis_direct');
    anfis_direct = S.anfis_direct;
end

U_pred = [st; mz];
[A, B, C] = anfis_direct_matrix(X, U_pred, vx);

x_next =  A * X + B * U_pred + C;

end
