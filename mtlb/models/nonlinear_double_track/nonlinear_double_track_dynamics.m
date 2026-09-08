function [vy_dot, r_dot, forces] = nonlinear_double_track_dynamics( ...
    vy, r, vx, delta, p)
%NONLINEAR_DOUBLE_TRACK_DYNAMICS Vectorized four-wheel lateral dynamics.
% Inputs may be scalars or equally sized column vectors.

wheelbase = p.lf + p.lr;
front_fraction = p.lr / wheelbase;
rear_fraction = p.lf / wheelbase;

fz_front_axle = p.m * p.g * front_fraction;
fz_rear_axle = p.m * p.g * rear_fraction;
fz_front_nominal = fz_front_axle / 2.0;
fz_rear_nominal = fz_rear_axle / 2.0;

% Contact-patch velocities include the yaw-induced difference between the
% left (+y) and right (-y) sides of each axle.
vx_fl = max(vx - 0.5 * p.track_front .* r, p.min_vx);
vx_fr = max(vx + 0.5 * p.track_front .* r, p.min_vx);
vx_rl = max(vx - 0.5 * p.track_rear .* r, p.min_vx);
vx_rr = max(vx + 0.5 * p.track_rear .* r, p.min_vx);

vy_front = vy + p.lf .* r;
vy_rear = vy - p.lr .* r;

alpha_fl = delta - atan2(vy_front, vx_fl);
alpha_fr = delta - atan2(vy_front, vx_fr);
alpha_rl = -atan2(vy_rear, vx_rl);
alpha_rr = -atan2(vy_rear, vx_rr);

% Initialize with the kinematic lateral-acceleration estimate, then update
% the load transfer and tyre forces to a self-consistent value.
ay = vx .* r;

for iteration = 1:p.load_transfer_iterations
    % Outside-minus-inside load difference. The factor two follows from
    % roll equilibrium: (Fz_out - Fz_in)*track/2 = m*ay*h.
    front_load_difference = 2.0 .* p.m .* ay .* p.cg_height .* ...
                            front_fraction ./ p.track_front;
    rear_load_difference = 2.0 .* p.m .* ay .* p.cg_height .* ...
                           rear_fraction ./ p.track_rear;

    % Preserve each axle's total normal load when a wheel approaches lift.
    front_limit = fz_front_axle - 2.0*p.min_normal_load;
    rear_limit = fz_rear_axle - 2.0*p.min_normal_load;
    front_load_difference = min(max(front_load_difference,-front_limit), ...
                                front_limit);
    rear_load_difference = min(max(rear_load_difference,-rear_limit), ...
                               rear_limit);

    fz_fl = max(0.5 * (fz_front_axle - front_load_difference), ...
                p.min_normal_load);
    fz_fr = max(0.5 * (fz_front_axle + front_load_difference), ...
                p.min_normal_load);
    fz_rl = max(0.5 * (fz_rear_axle - rear_load_difference), ...
                p.min_normal_load);
    fz_rr = max(0.5 * (fz_rear_axle + rear_load_difference), ...
                p.min_normal_load);

    df_fl = peak_force(fz_fl, fz_front_nominal, p.mu_front, ...
                       p.load_sensitivity);
    df_fr = peak_force(fz_fr, fz_front_nominal, p.mu_front, ...
                       p.load_sensitivity);
    dr_rl = peak_force(fz_rl, fz_rear_nominal, p.mu_rear, ...
                       p.load_sensitivity);
    dr_rr = peak_force(fz_rr, fz_rear_nominal, p.mu_rear, ...
                       p.load_sensitivity);

    fy_fl = pacejka_lateral(alpha_fl, p.tire_Bf, p.tire_Cf, df_fl);
    fy_fr = pacejka_lateral(alpha_fr, p.tire_Bf, p.tire_Cf, df_fr);
    fy_rl = pacejka_lateral(alpha_rl, p.tire_Br, p.tire_Cr, dr_rl);
    fy_rr = pacejka_lateral(alpha_rr, p.tire_Br, p.tire_Cr, dr_rr);

    front_body_force = (fy_fl + fy_fr) .* cos(delta);
    rear_body_force = fy_rl + fy_rr;
    ay = (front_body_force + rear_body_force) ./ p.m;
end

vy_dot = ay - vx .* r;
% The unequal front tyre forces also create a track-width moment because
% steering rotates part of each lateral tyre force into the body x-axis.
front_track_moment = 0.5*p.track_front .* (fy_fl-fy_fr) .* sin(delta);
r_dot = (p.lf.*front_body_force - p.lr.*rear_body_force + ...
         front_track_moment) ./ p.Iz;

if nargout > 2
    forces.alpha_fl = alpha_fl;
    forces.alpha_fr = alpha_fr;
    forces.alpha_rl = alpha_rl;
    forces.alpha_rr = alpha_rr;
    forces.fz_fl = fz_fl;
    forces.fz_fr = fz_fr;
    forces.fz_rl = fz_rl;
    forces.fz_rr = fz_rr;
    forces.fy_fl = fy_fl;
    forces.fy_fr = fy_fr;
    forces.fy_rl = fy_rl;
    forces.fy_rr = fy_rr;
    forces.front_track_moment = front_track_moment;
end
end

function d = peak_force(fz, fz_nominal, mu, load_sensitivity)
load_ratio = max(fz ./ fz_nominal, eps);
d = mu .* fz .* load_ratio .^ load_sensitivity;
end

function fy = pacejka_lateral(alpha, b, c, d)
fy = d .* sin(c .* atan(b .* alpha));
end
