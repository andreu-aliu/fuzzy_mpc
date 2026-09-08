function x_next = anfis_residuals_2(X, U, dt)
% X: [y vy psi r delta delta_dot]
% U: [vx steering_command]

validateattributes(U,{'numeric'},{'vector','numel',2,'finite'});
vx=U(1); st=U(2);

[A,B,Cc] = anfis_residuals_matrix(X,st,vx,dt);

x_next = A*X(:) + B*st + Cc;
end
