function x_next = anfis_dot(X, U, dt)
% X: [y vy psi r delta delta_dot]
% U: [vx steering_command]

% Unpack state and inputs
validateattributes(U,{'numeric'},{'vector','numel',2,'finite'});
vx=U(1); st=U(2);

[A,B,C] = anfis_dot_matrix(X,st,vx,dt);

x_next = A*X(:) + B*st + C;

end
