function x_next = nonlinear_double_track(X, U, dt)
%NONLINEAR_DOUBLE_TRACK One-step four-wheel lateral vehicle prediction.
% X: [y; vy; psi; r; delta; delta_dot]
% U: [vx, steering_command, Mz]

persistent params

if isempty(params)
    model_dir = fileparts(mfilename('fullpath'));
    parameter_file = fullfile(model_dir, 'nonlinear_double_track_params.mat');

    if isfile(parameter_file)
        saved = load(parameter_file, 'params');
        params = saved.params;
    else
        params = nonlinear_double_track_defaults();
    end
end

validateattributes(X, {'numeric'}, {'vector', 'numel', 6, 'finite'});
validateattributes(U, {'numeric'}, {'vector', 'numel', 3, 'finite'});
validateattributes(dt, {'numeric'}, {'scalar', 'real', 'finite', 'positive'});

x = X(:);
u = U(:);

vy = x(2);
psi = x(3);
r = x(4);
delta = min(max(x(5), -params.max_steering), params.max_steering);
delta_dot = x(6);

vx = u(1);
steering_command = min(max(u(2), -params.max_steering), ...
                       params.max_steering);
mz = u(3);

[vy_dot, r_dot] = nonlinear_double_track_dynamics( ...
    vy, r, vx, delta, mz, params);

y_dot = vx * sin(psi) + vy * cos(psi);
psi_dot = r;
delta_ddot = params.steering_wn^2 * (steering_command - delta) - ...
             2.0 * params.steering_zeta * params.steering_wn * delta_dot;

x_dot = [y_dot; vy_dot; psi_dot; r_dot; delta_dot; delta_ddot];
x_next = x + dt * x_dot;
x_next(5) = min(max(x_next(5), -params.max_steering), ...
                params.max_steering);
end
