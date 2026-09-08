function x_next = anfis_delta(X, U, dt)
% X: [y vy psi r delta delta_dot]
% U: [vx steering_command]
    
% Inputs are [longitudinal speed, steering command].
validateattributes(U,{'numeric'},{'vector','numel',2,'finite'});
vx = U(1);
st = U(2);

[A,B,C] = anfis_delta_matrix(X,st,vx,dt);

x_next = A*X(:) + B*st + C;

end
