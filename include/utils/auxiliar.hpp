#pragma once

#include <vector>
#include <cmath>
#include <limits>

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
    double psi;
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
    if (xq <= xs.front()) return ys.front();
    if (xq >= xs.back())  return ys.back();

    for (size_t i = 0; i < xs.size() - 1; ++i){
        if (xq >= xs[i] && xq <= xs[i+1]){
            double t = (xq - xs[i]) / (xs[i+1] - xs[i]);
            return ys[i] + t * (ys[i+1] - ys[i]);
        }
    }

    return ys.back(); // fallback
}


// ==============================
// build_reference_global
// ==============================
// Equivalent to MATLAB build_reference_global
// Output: vector<State>
inline std::vector<State> build_reference_global(
    const Trajectory& traj,
    const State& Xg,
    double dt,
    int Np)
{
    std::vector<State> X_ref(Np);

    // Extract arrays for interpolation
    std::vector<double> s_vec, x_vec, y_vec, psi_vec, vx_vec;
    s_vec.reserve(traj.size());
    x_vec.reserve(traj.size());
    y_vec.reserve(traj.size());
    psi_vec.reserve(traj.size());
    vx_vec.reserve(traj.size());

    for (const auto& p : traj){
        s_vec.push_back(p.s);
        x_vec.push_back(p.x);
        y_vec.push_back(p.y);
        psi_vec.push_back(p.psi);   // ✔ correct now
        vx_vec.push_back(p.vx);
    }

    int idx = find_closest_point(traj, Xg.x, Xg.y);
    double s_i = traj[idx].s;

    for (int i = 0; i < Np; ++i){

        if (s_i > s_vec.back())
            s_i = s_vec.back();

        double vx_i  = interp1(s_vec, vx_vec, s_i);
        double x_t   = interp1(s_vec, x_vec,  s_i);
        double y_t   = interp1(s_vec, y_vec,  s_i);
        double psi_t = interp1(s_vec, psi_vec, s_i);

        s_i += vx_i * dt;

        State Xi{};
        Xi.x = x_t;
        Xi.y = y_t;
        Xi.psi = psi_t;
        Xi.vx = vx_i;

        // rest zero (same as MATLAB)
        Xi.vy = 0.0;
        Xi.r  = 0.0;
        Xi.delta = 0.0;
        Xi.delta_dot = 0.0;
        // Xi.mz = 0.0;

        X_ref[i] = Xi;
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
    // X_local.mz = X_global.mz;

    return X_local;
}