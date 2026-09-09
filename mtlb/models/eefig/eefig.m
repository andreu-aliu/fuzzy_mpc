function x_next = eefig(X, U, dt)
%EEFIG Propagate the six-state vehicle model with frozen EEFig parameters.
% X: [y vy psi r delta delta_dot]
% U: [vx steering_command]

validateattributes(X, {'numeric'}, {'vector','numel',6,'finite'});
validateattributes(U, {'numeric'}, {'vector','numel',2,'finite'});

vx = U(1);
steering_command = U(2);
[A, B, C] = eefig_matrix(X, steering_command, vx, dt);
x_next = A * X(:) + B * steering_command + C;
end
