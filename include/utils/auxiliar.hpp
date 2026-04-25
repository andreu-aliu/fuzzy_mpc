#pragma once

#include <vector>
#include <cmath>
#include <limits>
#include <as_lib/utils/Profiler.hpp>

// ==============================
// DATA STRUCTURES
// ==============================

struct State{
    double x;
    double y;
    double psi;
    double vx;
    double vy;
    double r;
    double delta;
    double delta_dot;
};

struct Control{
    double steering;
    double mz;
};

struct TrajectoryPoint{
    double x;
    double y;
    double s;
    double k;
    double vx;
    double l;
    double r;
    double ax;
    double w;
};

using Trajectory = std::vector<TrajectoryPoint>;


// ==============================
// Helper: wrap angle
// ==============================
inline double wrapToPi(double angle){
    return std::atan2(std::sin(angle), std::cos(angle));
}


// ==============================
// find_closest_point
// ==============================
inline int find_closest_point(
    const Trajectory& traj,
    double x,
    double y)
{
    int idx = 0;
    double min_dist = std::numeric_limits<double>::max();

    for (size_t i = 0; i < traj.size(); ++i){
        double dx = traj[i].x - x;
        double dy = traj[i].y - y;
        double d2 = dx*dx + dy*dy;

        if (d2 < min_dist){
            min_dist = d2;
            idx = static_cast<int>(i);
        }
    }

    return idx;
}


// ==============================
// Linear interpolation helper
// ==============================
inline double interp1(
    const std::vector<double>& xs,
    const std::vector<double>& ys,
    double xq)
{
    if (xs.empty() || ys.empty() || xs.size() != ys.size()) {
        throw std::runtime_error("interp1: invalid input vectors");
    }

    if (xq <= xs.front()) return ys.front();
    if (xq >= xs.back())  return ys.back();

    for (size_t i = 0; i + 1 < xs.size(); ++i) {
        if (xq >= xs[i] && xq <= xs[i + 1]) {
            double ds = xs[i + 1] - xs[i];

            if (std::abs(ds) < 1e-9) {
                return ys[i];
            }

            double t = (xq - xs[i]) / ds;
            return ys[i] + t * (ys[i + 1] - ys[i]);
        }
    }

    return ys.back();
}


// ==============================
// build_reference_global
// ==============================
// Output: vector<State>
inline std::vector<State> build_reference_global(
    const Trajectory& traj,
    const State& Xg,
    double dt,
    int Np)
{
    PROFC_NODE_

    std::vector<State> X_ref(Np);

    if (traj.size() < 2) {
        std::cerr << "Warning: Trajectory has less than 2 points. Returning empty reference." << std::endl;
        return X_ref; // not enough data
    }

    // 1. Precompute psi from geometry
    std::vector<double> psi_vec(traj.size());

    for (size_t i = 0; i < traj.size(); ++i)
    {
        double dx, dy;

        if (i == 0) {
            dx = traj[i+1].x - traj[i].x;
            dy = traj[i+1].y - traj[i].y;
        }
        else if (i == traj.size() - 1) {
            dx = traj[i].x - traj[i-1].x;
            dy = traj[i].y - traj[i-1].y;
        }
        else {
            dx = traj[i+1].x - traj[i-1].x;
            dy = traj[i+1].y - traj[i-1].y;
        }

        double norm = std::hypot(dx, dy);

        if (norm < 1e-9) {
            psi_vec[i] = (i > 0) ? psi_vec[i-1] : 0.0;
        } else {
            psi_vec[i] = std::atan2(dy, dx);
        }
    }

    // 2. Find starting point
    int idx = find_closest_point(traj, Xg.x, Xg.y);
    double s_i = traj[idx].s;

    size_t k = static_cast<size_t>(idx);

    // 3. Build reference
    for (int i = 0; i < Np; ++i)
    {
        // Clamp to end
        if (s_i >= traj.back().s)
        {
            const auto& p = traj.back();

            X_ref[i] = {
                p.x,
                p.y,
                psi_vec.back(),
                p.vx,
                0.0, 
                p.r,
                0.0, 0.0
            };
            continue;
        }

        // Advance cursor
        while (k + 1 < traj.size() && traj[k + 1].s < s_i)
            ++k;

        const auto& p0 = traj[k];
        const auto& p1 = traj[std::min(k + 1, traj.size() - 1)];

        double ds = p1.s - p0.s;

        double t = 0.0;
        if (std::abs(ds) > 1e-9) {
            t = (s_i - p0.s) / ds;
            t = std::clamp(t, 0.0, 1.0);
        }

        // Interpolate position and velocity
        double x_t  = p0.x  + t * (p1.x  - p0.x);
        double y_t  = p0.y  + t * (p1.y  - p0.y);
        double vx_t = p0.vx + t * (p1.vx - p0.vx);
        double r_t  = p0.r + t * (p1.r - p0.r);

        // Interpolate angle (safe wrap)
        double dpsi = wrapToPi(psi_vec[k+1] - psi_vec[k]);
        double psi_t = wrapToPi(psi_vec[k] + t * dpsi);

        // Build state
        X_ref[i] = {
            x_t,
            y_t,
            psi_t,
            vx_t,
            0.0, 
            r_t,
            0.0, 0.0
        };

        // Advance along trajectory
        s_i += vx_t * dt;

        if (!std::isfinite(s_i))
            s_i = p0.s;
    }

    return X_ref;
}

// ==============================
// global_to_local_state
// ==============================
// Input: global state
// Output: local MPC state + vx
inline State global_to_local_state(
    const State& X_global,
    const State& X_ref)
{
    // Relative position
    double dx = X_global.x - X_ref.x;
    double dy = X_global.y - X_ref.y;

    // Rotation (global → local frame)
    double x_local_pos =  std::cos(X_ref.psi)*dx + std::sin(X_ref.psi)*dy;
    double y_local_pos = -std::sin(X_ref.psi)*dx + std::cos(X_ref.psi)*dy;

    // Heading error
    double psi_rel = wrapToPi(X_global.psi - X_ref.psi);

    State X_local{};

    // Position in local frame
    X_local.x = x_local_pos;
    X_local.y = y_local_pos;

    // Orientation
    X_local.psi = psi_rel;

    // Velocities
    X_local.vx = X_global.vx;
    X_local.vy = X_global.vy;
    X_local.r  = X_global.r;

    // Actuator states
    X_local.delta     = X_global.delta;
    X_local.delta_dot = X_global.delta_dot;

    return X_local;
}