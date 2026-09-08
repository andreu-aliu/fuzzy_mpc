function params = nonlinear_double_track_defaults()
%NONLINEAR_DOUBLE_TRACK_DEFAULTS Nominal four-wheel planar-model parameters.

params.model = "nonlinear double track";

% Vehicle properties
params.m = 215.0;       % Mass [kg]
params.Iz = 188.0;      % Yaw inertia [kg m^2]
params.lf = 0.765;      % CoG to front axle [m]
params.lr = 0.765;      % CoG to rear axle [m]

% Geometry used for left/right wheel kinematics and lateral load transfer.
% Replace these nominal values with measured CAT vehicle geometry.
params.track_front = 1.20;  % Front track width [m]
params.track_rear = 1.20;   % Rear track width [m]
params.cg_height = 0.28;    % CoG height [m]
params.g = 9.81;            % Gravity [m/s^2]

% Per-tyre simplified Pacejka law:
% Fy = D(Fz)*sin(C*atan(B*alpha)).
params.tire_Bf = 10.5507;
params.tire_Cf = 1.2705;
params.tire_Br = 10.5507;
params.tire_Cr = 1.2705;
params.mu_front = 1.05;
params.mu_rear = 1.22;
params.load_sensitivity = -0.10;

% Steering-actuator dynamics
params.steering_wn = 16.0;
params.steering_zeta = 0.5;

% Numerical settings
params.min_vx = 0.5;
params.min_normal_load = 1.0;
params.load_transfer_iterations = 3;
params.max_steering = 0.45;
end
