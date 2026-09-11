function X_next = sim_nonlinear_bicycle(X,U,vx_next,dt)
%SIM_NONLINEAR_BICYCLE Propagate the global nonlinear bicycle plant once.
% X       = [x y psi vx vy r delta delta_dot]
% U       = steering command [rad]
% vx_next = prescribed longitudinal speed at the next sample [m/s]
% dt      = integration step [s]

validateattributes(X,{'numeric'}, ...
    {'vector','numel',8,'real','finite'},mfilename,'X');
validateattributes(U,{'numeric'}, ...
    {'vector','nonempty','real','finite'},mfilename,'U');
validateattributes(vx_next,{'numeric'}, ...
    {'scalar','real','finite'},mfilename,'vx_next');
validateattributes(dt,{'numeric'}, ...
    {'scalar','real','finite','positive'},mfilename,'dt');

state = X(:);
vx = state(4);
vy = state(5);
psi = state(3);

% Reuse the identified six-state model so this simulator always uses the
% same fitted/default tyre and actuator parameters as model comparison.
local_state = [state(2);state(5);state(3);state(6);state(7);state(8)];
local_next = nonlinear_bicycle(local_state,[vx;U(1)],dt);

% nonlinear_bicycle already advances global y, psi, lateral dynamics, and
% the steering actuator. Add the missing global x kinematics here.
x_dot = vx*cos(psi)-vy*sin(psi);
x_next = state(1)+dt*x_dot;

X_next = [x_next;local_next(1);local_next(3);vx_next; ...
    local_next(2);local_next(4);local_next(5);local_next(6)].';
end
