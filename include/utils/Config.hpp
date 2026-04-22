#pragma once

#include <string>

struct Config{

    bool verbose;
    bool save_debug;
    bool profile;
    std::string debug_path;

    std::string ws_path;
    std::string share_path;

    // MPC
    struct MPC{
        double Ts;
        int n_horizon;      // prediction horizon        
        double latency;     // actuation latency [s]

        // scalings
        double scale_y;
        double scale_vy;
        double scale_phi;
        double scale_r;
        double scale_st;
        double scale_dst;
        double scale_mz;
        double scale_dmz;
        
        // state weights
        double q_lat;
        double q_vy;
        double q_phi;
        double q_r;
        double q_delta;
        double q_delta_dot;
        // last point state weights
        double p_lat;
        double p_vy;
        double p_phi;
        double p_r;
        double p_delta;
        double p_delta_dot;
        // control weights
        double r_st;
        double r_mz;
        // Change in control weights
        double rd_st;
        double rd_mz;

    }mpc;

    // Car
    struct Car{
        double m;   // mass
        double I;   // yaw inertia
        double Lf;  // distance from CG to front axle
        double Lr;  // distance from CG to rear axle
        double Bf;  // front stiffness parameter B
        double Br;  // rear stiffness parameter B
        double Cf;  // front stiffness parameter C
        double Cr;  // rear stiffness parameter C
        double Df;  // front stiffness parameter D
        double Dr;  // rear stiffness parameter D
        double steering_damp;
        double steering_omega;
    }car;

    // Topics
    struct Topics{
        std::string in_state;        // input car state
        std::string in_planner;      // input trajectory from planner
        std::string in_velocity;     // input velocity reference from PID
        std::string out_steering;    // output steering command
        std::string out_duration;    // output duration of the MPC computation

        struct Visualization{
            std::string predictedSteering; // predicted steering visualization
            std::string predictedPath;     // predicted path visualization
            std::string predictedHeading;  // predicted heading visualization
            std::string referencePath;     // reference path visualization
        }vis;

        struct Debug{
            std::string out_lateral_error; // lateral error output for debugging
            std::string out_heading_error; // heading error output for debugging
            std::string out_yaw_rate_ref;  // yaw rate reference output for debugging
        }debug;

    }topics;
    
    // Singleton pattern
    static Config& getInstance() {
        static Config instance;
        return instance;
    }

    private:
    Config() = default;

    // Delete copy/move so extra instances can't be created/moved.
    Config(const Config&) = delete;
    Config& operator=(const Config&) = delete;
    Config(Config&&) = delete;
    Config& operator=(Config&&) = delete;
};
