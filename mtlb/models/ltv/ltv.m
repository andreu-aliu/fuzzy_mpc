function x_next = ltv(X, U, dt)
% X: [y vy psi r delta delta_dot]
% U: [vx steering_command]

validateattributes(X,{'numeric'},{'vector','numel',6,'finite'});
validateattributes(U,{'numeric'},{'vector','numel',2,'finite'});
validateattributes(dt,{'numeric'},{'scalar','real','finite','positive'});

vx = U(1);
steering_command = U(2);
[A,B,C] = ltv_matrix(X,steering_command,vx,dt);
x_next = A*X(:) + B*steering_command + C;
end
