#pragma once

#include <string>

struct Config{

    bool profile;
    std::string ws_path;
    std::string node_path;
    std::string share_path;

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
    }car;

    // MPC
    struct MPC{
        int n_horizon;      // prediction horizon
        int n_planning;     // number of points from the planner to consider
        int n_states;       // number of states
        int n_controls;     // number of controls
        double Ts;          // sampling time
        double disc;        // discretization method (0: Euler, 1: Tustin)
        double latency;     // actuation latency [s]
        
        bool verbose;       // verbosity flag
        std::string debug_path; // TODO: Delete, not used
        bool save_debug;
        
        // state weights
        double q_lat;
        double q_vy;
        double q_phi;
        double q_r;
        double q_delta;
        // control weights
        double r_delta;
        // last point state weights
        double p_lat;
        double p_vy;
        double p_phi;
        double p_r;
        double p_delta;
        // initialization weights
        double q_lat_init;
        double q_phi_init;
        double q_delta_init;
        double r_delta_init;

        double vx_to_finish_init; // velocity to reach the end of the trajectory for initialization [m/s]
    }mpc;

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
            std::string actualPath;        // actual path visualization
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
