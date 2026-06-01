function x_next = anfis_residuals_2(X, U, dt)
% X: [y vy psi r delta delta_dot]
% U: [vx st Mtv]

C = num2cell(U);
[vx, st, mz] = deal(C{:});

U_pred = [st; mz];
[A, B, Cc] = anfis_residuals_matrix(X, U_pred, vx);

x_next = A * X + B * U_pred + Cc;
end

