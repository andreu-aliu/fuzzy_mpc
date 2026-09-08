function x_next = nonlinear_bicycle(X, U, dt)
%NONLINEAR_BICYCLE One-step nonlinear lateral vehicle prediction.
% X: [y; vy; psi; r; delta; delta_dot]
% U: [vx, steering_command, Mz]

persistent params

if isempty(params)
    model_dir = fileparts(mfilename('fullpath'));
    parameter_file = fullfile(model_dir, 'nonlinear_bicycle_params.mat');

    if isfile(parameter_file)
        saved = load(parameter_file, 'params');
        params = saved.params;
    else
        params = nonlinear_bicycle_defaults();
    end
end

validateattributes(X, {'numeric'}, {'vector', 'numel', 6, 'finite'});
validateattributes(U, {'numeric'}, {'vector', 'numel', 3, 'finite'});
validateattributes(dt, {'numeric'}, {'scalar', 'real', 'finite', 'positive'});

x = X(:);
u = U(:);

% State and inputs
vy = x(2);
psi = x(3);
r = x(4);
delta = min(max(x(5), -params.max_steering), params.max_steering);
delta_dot = x(6);

vx = u(1);
steering_command = min(max(u(2), -params.max_steering), ...
                       params.max_steering);
mz = u(3);
vx_slip = max(abs(vx), params.min_vx);

% Exact nonlinear bicycle slip angles
alpha_f = delta - atan2(vy + params.lf * r, vx_slip);
alpha_r = -atan2(vy - params.lr * r, vx_slip);

% Nonlinear axle lateral forces
fyf = params.tire_Df * sin(params.tire_Cf * ...
      atan(params.tire_Bf * alpha_f));
fyr = params.tire_Dr * sin(params.tire_Cr * ...
      atan(params.tire_Br * alpha_r));

% Nonlinear lateral dynamics and global-frame kinematics
y_dot = vx * sin(psi) + vy * cos(psi);
vy_dot = (fyf * cos(delta) + fyr) / params.m - vx * r;
psi_dot = r;
r_dot = (params.lf * fyf * cos(delta) - params.lr * fyr + mz) / ...
        params.Iz;

% Second-order steering actuator
delta_ddot = params.steering_wn^2 * (steering_command - delta) - ...
             2.0 * params.steering_zeta * params.steering_wn * delta_dot;

x_dot = [y_dot; vy_dot; psi_dot; r_dot; delta_dot; delta_ddot];
x_next = x + dt * x_dot;
x_next(5) = min(max(x_next(5), -params.max_steering), ...
                params.max_steering);
end
