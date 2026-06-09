function x = clamp_local_state(x)
% Clamp local vehicle state to avoid propagation blow-ups.
% x: [y vy psi r delta delta_dot]

x = x(:);
if numel(x) ~= 6
    error('clamp_local_state expects 6-state vector [y vy psi r delta delta_dot]');
end

% y local [m]
x(1) = min(max(x(1), -50.0), 50.0);

% vy [m/s]
x(2) = min(max(x(2), -10.0), 10.0);

% psi [rad] wrapped to [-pi, pi)
x(3) = mod(x(3) + pi, 2*pi) - pi;

% r [rad/s]
x(4) = min(max(x(4), -5.0), 5.0);

% delta [rad]
x(5) = min(max(x(5), -0.45), 0.45);

% delta_dot [rad/s]
x(6) = min(max(x(6), -8.0), 8.0);
end

