function params = nonlinear_bicycle_defaults()
%NONLINEAR_BICYCLE_DEFAULTS Nominal nonlinear bicycle-model parameters.

params.model = "nonlinear bicycle";

% Vehicle properties
params.m = 220.0;       % Mass [kg]
params.Iz = 188.0;      % Yaw inertia [kg m^2]
params.lf = 0.765;      % CoG to front axle [m]
params.lr = 0.765;      % CoG to rear axle [m]

% Simplified Pacejka tyre law: Fy = D*sin(C*atan(B*alpha)).
% D represents the complete axle (two tyres).
params.tire_Bf = 10.5507;
params.tire_Cf = 1.2705;
params.tire_Df = 2.0 * 1104.0;  % Front axle peak force [N]
params.tire_Br = 10.5507;
params.tire_Cr = 1.2705;
params.tire_Dr = 2.0 * 1281.5;  % Rear axle peak force [N]

% Steering-actuator dynamics
params.steering_wn = 16.0;       % Natural frequency [rad/s]
params.steering_zeta = 0.5;      % Damping ratio [-]

% Numerical limits
params.min_vx = 0.5;             % Slip-angle denominator floor [m/s]
params.max_steering = 0.45;      % Steering-angle limit [rad]
end
