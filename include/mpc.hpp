#pragma once

#include <vector>
#include <numeric>
#include <fstream>
#include <cmath>
#include <array>
#include <chrono>
#include <algorithm> 
#include <filesystem>

#include <eigen3/Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include "utils/kdtree.hpp"
#include "utils/solver.hpp"
#include "utils/Config.hpp"
#include <as_lib/utils/Profiler.hpp>

class Point : public std::array<double, 2> {
    public:
    static const int DIM = 2;
};

class MPC {
       
  private:
    Config& cfg;
    Solver solver;

    // KDTree for the trajectory
    kdt::KDTree<Point> planner_tree_;

    // Optimization matrices
    vector<Eigen::MatrixXd> A, C;
    vector<Eigen::VectorXd> B, D;
    vector<Eigen::MatrixXd> C_powers;
    Eigen::MatrixXd P, R, R_, S, T, H, I;
    Eigen::VectorXd x0, x_ref, x_prev, q_diag, g;
    Eigen::MatrixXd QS;
    Eigen::MatrixXd S_transpose;
    Eigen::MatrixXd StQS;

    // Solution matrix
    Eigen::VectorXd delta_u_opt, pred_states;

    // Planner and state matrices
    Eigen::MatrixXd planner;
    Eigen::VectorXd car_state;

    // Velocity and heading references
    vector<double> vx, transformed_headings;
    Eigen::VectorXd pred_velocities; 

    // Variables
    int n_states_, n_controls_, n_horizon_, n_planning_;
    double Ts_, disc_;
    bool verbose_;

    // Car parameters
    double m_, Iz_, lf_, lr_, Cf_, Cr_;

    // Steering dynamics parameters
    double damp_, omega_, alpha_delta_dot, delta_dot_filtered, last_delta;

    // Tire stiffness parameters
    double Bf, Br, Cf, Cr, Df, Dr;

    // Weights
    int latency_;
    double q_lat_, q_vy_, q_phi_, q_r_, q_delta_, r_delta_, p_lat_, p_vy_, p_phi_, p_r_, p_delta_;
    double q_lat_init_, q_phi_init_, q_delta_init_, r_delta_init_, vx_to_finish_init_;

    // References auxiliar variables
    vector<Vector2d> planner_traj_frame;
    double reference_x, reference_y, reference_heading, cos_ref, sin_ref;

    // First iteration for initialization porpuses
    bool firstIteration = true;


    /////////////////////////////////////////////////////////////////////////////
    //-------------------------- Auxiliar functions  --------------------------//

    // Create kd-tree from the planner points
    void createKDTree(){
        vector<Point> tree;
        for (size_t i = 0; i < planner.rows(); i++)
        {
            Point p;
            p[0] = planner(i, 0);
            p[1] = planner(i, 1);
            tree.push_back(p);
        }
        planner_tree_.build(tree);
    }

    // Calculate the heading angle based on x and y differences
    double calcHeading(const double &x, const double &y){
        if (x >= 0)
            return atan2(y, x);
        else if (y > 0)
            return M_PI - atan2(y, fabs(x));
        else
            return -M_PI + (atan2(fabs(y), fabs(x)));
    }

    // Ensure continuity of angles
    double continuous(const double &psi, const double &psi_last){
        double diff = psi - psi_last;

        int k = 0;
        while (abs(diff) > (2 * M_PI - 1)){
            if (k > 12){
                k = 0;
                break;
            }
            if (k > 0){
                k = -k;
            }else if (k <= 0){
                k = -k + 1;
            }
            diff = psi - psi_last + 2 * M_PI * k;
        }
        return psi + 2 * M_PI * k;
    }
    
    // Safety function for steering command
    double steeringSafety(const double &steering){
        if (isnan(steering))
            return 0.0;
        else if (car_state(3) < 0.3)
            return 0.0;
        else if (steering > 25 * M_PI / 180.0)
            return 25 * M_PI / 180.0;
        else if (steering < -25 * M_PI / 180.0)
            return -25 * M_PI / 180.0;
        else
            return steering;
    }

    // Debugging function to append a single row to a CSV file
    void appendSingleRowToCSV(const std::vector<double>& vector, std::string vectorname){
        std::string filename = cfg.mpc.debug_path + vectorname + ".csv";

        // Open the CSV file in append mode
        std::ofstream file(filename, std::ios::app);

        if (!file.is_open())
        {
            std::cerr << "Error: Could not open csv for writing" << std::endl;
            return;
        }

        // Write the 5x1 vector as a new row in the CSV
        for (size_t i = 0; i < vector.size() - 1; ++i)
            file << vector[i] << ",";
        file << vector[vector.size() - 1] << "\n";

        file.close();
        std::cout << "[fuzzy_mpc] " << vectorname << " saved to " << filename << std::endl;
    }

    // Debugging function to append a single row to a CSV file
    void appendSingleRowToCSV(const Eigen::VectorXd& vector, std::string vectorname){
        std::string filename = cfg.mpc.debug_path + vectorname + ".csv";        
        // Open the CSV file in append mode
        std::ofstream file(filename, std::ios::app);
        
        if (!file.is_open())
        {
            std::cerr << "Error: Could not open csv for writing" << std::endl;
            return;
        }
        
        // Write the 5x1 vector as a new row in the CSV
        for (size_t i = 0; i < vector.size() - 1; ++i)
            file << vector(i) << ",";
        file << vector(vector.size() - 1) << "\n";
        
        file.close();
        std::cout << "[fuzzy_mpc] " << vectorname << " saved to " << filename << std::endl;
    }

  public:   
    // Constructor
    MPC(): cfg(Config::getInstance()) {}

    void initialize(){
        
        std::cout << "Initializing MPC..." << std::endl;
        // Save recurrent parameters
        n_states_ = cfg.mpc.n_states;
        n_controls_ = cfg.mpc.n_controls;
        n_horizon_ = cfg.mpc.n_horizon;
        n_planning_ = cfg.mpc.n_planning;
        Ts_ = cfg.mpc.Ts;
        disc_ = cfg.mpc.disc;
        verbose_ = cfg.mpc.verbose;

        // Car parameters
        m_ = cfg.car.m;
        Iz_ = cfg.car.I;
        lf_ = cfg.car.Lf;
        lr_ = cfg.car.Lr;
        // Cf_ = cfg.car.Cf;
        // Cr_ = cfg.car.Cr;

        Bf = cfg.car.Bf;
        Br = cfg.car.Br;
        Cf = cfg.car.Cf;
        Cr = cfg.car.Cr;
        Df = cfg.car.Df;
        Dr = cfg.car.Dr;

        // Initialize mpc matrices
        x0.resize(n_states_);
        x_ref.resize(n_states_ * n_horizon_);
        x_prev.resize(n_states_ * n_horizon_);
        R.resize(n_controls_, n_controls_);
        R_.resize(n_horizon_ * n_controls_, n_horizon_ * n_controls_);
        S.resize(n_horizon_ * n_states_, n_horizon_);
        T.resize(n_horizon_ * n_states_, n_states_);
        H.resize(n_horizon_, n_horizon_);
        g.resize(n_horizon_);
        QS.resize(n_horizon_ * n_states_, n_horizon_);
        S_transpose.resize(n_horizon_, n_horizon_ * n_states_);
        StQS.resize(n_horizon_, n_horizon_);
        delta_u_opt.resize(n_horizon_ * n_controls_);
        pred_states.resize(n_horizon_ * n_states_);
        planner_traj_frame.resize(n_horizon_ + 1);
        transformed_headings.resize(n_horizon_ + 1);
        
        A.resize(n_horizon_);
        B.resize(n_horizon_);
        C.resize(n_horizon_);
        D.resize(n_horizon_);
        C_powers.resize(n_horizon_ + 1);

        for (size_t i = 0; i < n_horizon_; ++i){
            A[i].resize(n_states_, n_states_);
            B[i].resize(n_states_);
            C[i].resize(n_states_, n_states_);
            D[i].resize(n_states_);
        }

        for (size_t i = 0; i <= n_horizon_; ++i){
            C_powers[i].resize(n_states_, n_states_);
        }

        vx.resize(n_horizon_+1);

        S = Eigen::MatrixXd::Zero(n_horizon_ * n_states_, n_horizon_);

        // Planner and State matrices
        planner.resize(n_planning_, 9);
        car_state.resize(7);
        pred_velocities.resize(n_planning_);

        I = Eigen::MatrixXd::Identity(n_states_, n_states_);

        // Initialize weights
        latency_ = cfg.mpc.latency;
        q_lat_ = cfg.mpc.q_lat;
        q_vy_ = cfg.mpc.q_vy;
        q_phi_ = cfg.mpc.q_phi;
        q_r_ = cfg.mpc.q_r;
        q_delta_ = cfg.mpc.q_delta;
        r_delta_ = cfg.mpc.r_delta;
        p_lat_ = cfg.mpc.p_lat;
        p_vy_ = cfg.mpc.p_vy;
        p_phi_ = cfg.mpc.p_phi;
        p_r_ = cfg.mpc.p_r;
        p_delta_ = cfg.mpc.p_delta;

        // Weights for the start of the run
        q_lat_init_ = cfg.mpc.q_lat_init;
        q_phi_init_ = cfg.mpc.q_phi_init;
        q_delta_init_ = cfg.mpc.q_delta_init;
        r_delta_init_ = cfg.mpc.r_delta_init;
        vx_to_finish_init_ = cfg.mpc.vx_to_finish_init;

        createWeights(true);

        firstIteration = true;

        double max_steering = 30.0 * M_PI / 180.0; // rad
        double max_steering_dot = 80.0 * M_PI / 180.0; // rad/s

        damp_ = 0.5;
        omega_ = 16; // Natural freq = 16 rad/s
        alpha_delta_dot = 1 - exp(-2 * M_PI * 5.0); // 5 Hz cutoff frequency
        last_delta = 0.0;
        delta_dot_filtered = 0.0;

        solver.setParams(max_steering, max_steering_dot, n_states_, n_horizon_, n_controls_, verbose_);
        std::cout << "MPC initialized" << std::endl;
    }

    ~MPC() = default;

    // Setters
    void setState(Eigen::VectorXd car_state){
        this->car_state = car_state;
    }

    void setPlanner(Eigen::MatrixXd planner){
        this->planner = planner;
        createKDTree();
        if (firstIteration){
            firstIteration = false;
        }
    }
   
    void setVels(Eigen::VectorXd vels){
        
        this->pred_velocities = vels;
    }


    /////////////////////////////////////////////////////////////////////////
    //-------------------------- MPC functions  ---------------------------//

    // Select the references and convert it into the car's frame
    void findReferences(){
        PROFC_NODE_

        vector<int> selected_index(n_horizon_ + 1);
        vector<double> heading_track(n_horizon_ + 1);
        vector<double> consec_heading_track(n_horizon_ + 1);

        Point p;
        p[0] = car_state(0);
        p[1] = car_state(1);
        
        // Planner point closest to the car
        int next_state = planner_tree_.nnSearch(p);
        selected_index[0] = next_state;
        
        // Precompute cos and sin of reference heading
        reference_x = planner(selected_index[0], 0);
        reference_y = planner(selected_index[0], 1);
        reference_heading = calcHeading(planner(selected_index[0] + 1, 0) - planner(selected_index[0], 0),
                                        planner(selected_index[0] + 1, 1) - planner(selected_index[0], 1));
        reference_heading = continuous(reference_heading, 0.0);
        cos_ref = cos(reference_heading);
        sin_ref = sin(reference_heading);

        Eigen::VectorXd x_ref_aux = Eigen::VectorXd::Zero(n_states_ * (n_horizon_ + 1));
        for (size_t i = 0; i < n_states_ * (n_horizon_ + 1); ++i){ // TODO: Really necessary?
            x_ref_aux(i) = 0.0;
        }

        for (size_t i = 0; i <= n_horizon_; ++i){

            double x = planner(selected_index[i], 0);
            double y = planner(selected_index[i], 1);
            // vx[i] = planner(selected_index[i], 4);
            vx[i] = pred_velocities(selected_index[i]);
            
            // Calculate the heading based on predicted position
            int go2idx = static_cast<int>(vx[i] * Ts_ / disc_);
            int next_idx = selected_index[i] + go2idx;//std::min(selected_index[i] + go2idx, planner.rows() - 1.0); TODO: Is it safe?
            double x_ = planner(next_idx, 0);
            double y_ = planner(next_idx, 1);
            
            // Calculate the heading of the trajectory
            double diff_x = x_ - x;
            double diff_y = y_ - y;
            double mu_ = calcHeading(diff_x, diff_y);

            mu_ = (i == 0) ? continuous(mu_, car_state(2)) : continuous(mu_, heading_track[i - 1]);

            heading_track[i] = mu_;
            
            // Predicted next position
            p[0] = x + vx[i] * cos(mu_) * Ts_;
            p[1] = y + vx[i] * sin(mu_) * Ts_;
            
            // Find the closest trajectory point to the predicted position
            next_state = planner_tree_.nnSearch(p);
            next_state = (next_state <= selected_index[i]) ? selected_index[i] + 1 : next_state;

            if (i != n_horizon_)
                selected_index[i + 1] = next_state; // std::min(next_state, static_cast<int>(planner.rows() - 2.0));

            // Convert planner point to car frame
            double translated_x = x - reference_x;
            double translated_y = y - reference_y;
            double rotated_x = translated_x * cos_ref + translated_y * sin_ref;
            double rotated_y = -translated_x * sin_ref + translated_y * cos_ref;
            
            planner_traj_frame[i] = Vector2d(rotated_x, rotated_y);
            
            // Calculate the heading of the trajectory, with two consecutive points
            double consec_diff_x = planner(selected_index[i] + 1, 0) - planner(selected_index[i], 0);
            double consec_diff_y = planner(selected_index[i] + 1, 1) - planner(selected_index[i], 1);
            double consec_mu = calcHeading(consec_diff_x, consec_diff_y);
            
            consec_mu = (i == 0) ? continuous(consec_mu, car_state(2)) : continuous(consec_mu, consec_heading_track[i - 1]);

            consec_heading_track[i] = consec_mu;
            
            // Update heading to trajectory frame
            transformed_headings[i] = consec_heading_track[i] - reference_heading;
            transformed_headings[i] = (i == 0) ? continuous(transformed_headings[i], reference_heading) :
                                                continuous(transformed_headings[i], transformed_headings[i - 1]);
                                                
            // Fill the x_ref matrix
            Eigen::MatrixXd ref_matrix(n_states_, 1);
            ref_matrix << planner_traj_frame[i](1),     // y
                        0,                              // vy
                        transformed_headings[i],        // Headin respect to reference frame
                        planner(selected_index[i], 8),  // Yaw rate
                        0,                              // delta
                        0;                              // delta dot

            x_ref_aux.block(i * n_states_, 0, n_states_, 1) = ref_matrix; // TODO: Really necessary?
        }

        // Fill x_ref vector [x_ref1 x_ref2 ... x_refN]^T, without x_ref0 which is irrelevant
        x_ref = x_ref_aux.block(n_states_, 0, n_states_ * n_horizon_, 1);

        if (cfg.mpc.save_debug){
            appendSingleRowToCSV(x_ref, "x_ref");
        }
        
        // Print the first 10 values of x_ref
        if (verbose_) std::cout << "First 10 values of x_ref: \n" << x_ref.block(0, 0, 10, 1).transpose() << std::endl;
    }

    // Create matrixes P, R and R_ of weithgs and vector q_diag
    void createWeights(bool init_state){
        PROFC_NODE_

        // Create matrix R
        R = Eigen::MatrixXd::Zero(n_controls_, n_controls_);
        if (init_state)
            R(0, 0) = 50.0; //q_delta_init_;        // TODO: Why hard coded?
        else
            R(0, 0) = 0.0; //q_delta_;

        // Q_ matrix diagonal
        Eigen::VectorXd diag_values(n_states_);
        if (init_state)
            diag_values << q_lat_init_, q_vy_, q_phi_init_, q_r_, q_delta_init_, r_delta_init_;
        else
            diag_values << q_lat_, q_vy_, q_phi_, q_r_, q_delta_, r_delta_;

        q_diag.resize(n_states_ * n_horizon_);
        for (size_t i = 0; i < (n_horizon_-1); ++i)
            q_diag.segment(i * n_states_, n_states_) = diag_values;

        q_diag.segment((n_horizon_-1)*n_states_, n_states_) << p_lat_, p_vy_, p_phi_, p_r_, p_delta_, r_delta_;

        // Create matrix R_
        R_ = Eigen::MatrixXd::Zero(n_horizon_ * n_controls_, n_horizon_ * n_controls_);
        for (size_t i = 0; i < n_horizon_; ++i)
            R_.block(i * n_controls_, i * n_controls_, n_controls_, n_controls_) = R;

        // Print the first 20x20 values of R_
        if (verbose_)
            std::cout << "First 20x20 values of R_: \n" << R_.block(0, 0, 20, 20) << std::endl;
    }

    // Calculate matrices A, B, C, D, S, T, H and g
    void createModelMatrices() {
        // This function has four main things to do
        // 1 - Fill x0 vector (y, vy, phi, r, delta)
        // 2 - Creates A and B based on car parameters (x_dot = Ax + Bu)
        // 3 - Discretize the system to create C and D using Exponential matrix method
        // 4 - Build S and T matrices to get rid of state variables
        // 5 - Finally build H and g matrices to solve the optimization problem

        PROFC_NODE_

        // Convert car state to reference frame
        double translated_x = car_state(0) - reference_x;
        double translated_y = car_state(1) - reference_y;
        double rotated_y = -translated_x * sin_ref + translated_y * cos_ref;
        double heading_rotated = car_state(2) - reference_heading;
        heading_rotated = continuous(heading_rotated, 0.0);

        if (firstIteration)
            last_delta = car_state(6);

        delta_dot_filtered += alpha_delta_dot * ((car_state(6) - last_delta) / Ts_ - delta_dot_filtered);

        // Fill initial/measured state
        x0(0) = rotated_y;           // y
        x0(1) = car_state(4);        // vy
        x0(2) = heading_rotated;     // heading respect the reference frame
        x0(3) = car_state(5);        // yaw rate
        x0(4) = car_state(6);        // Steering
        x0(5) = delta_dot_filtered;  // Filtered steering dot

        last_delta = car_state(6);

        if (cfg.mpc.save_debug){
            appendSingleRowToCSV(x0, "x0");
        }

        vector<double> prev_vy(n_horizon_);
        vector<double> prev_phi(n_horizon_);
        vector<double> prev_r(n_horizon_);
        vector<double> prev_delta(n_horizon_);

        prev_vy[0] = car_state(4);
        prev_phi[0] = heading_rotated;
        prev_r[0] = car_state(5);
        prev_delta[0] = car_state(6);

        if (firstIteration){
            for (size_t i = 1; i < n_horizon_; ++i){
                prev_vy[i] = car_state(4);
                prev_phi[i] = heading_rotated;
                prev_r[i] = car_state(5);
                prev_delta[i] = min(max(car_state(6), -25.0 * M_PI / 180.0), 25.0 * M_PI / 180.0);
            }
            firstIteration = false;
        }
        else{
            for (size_t i = 1; i < n_horizon_ - 1; ++i){
                prev_vy[i] = pred_states((i + 1) * n_states_ + 1);
                prev_phi[i] = pred_states((i + 1) * n_states_ + 2);
                prev_r[i] = pred_states((i + 1) * n_states_ + 3);
                prev_delta[i] = min(max(pred_states((i + 1) * n_states_ + 4), -25.0 * M_PI / 180.0), 25.0 * M_PI / 180.0);
            }
            prev_vy[n_horizon_ - 1] = pred_states((n_horizon_ - 1) * n_states_ + 1);
            prev_phi[n_horizon_ - 1] = pred_states((n_horizon_ - 1) * n_states_ + 2);
            prev_r[n_horizon_ - 1] = pred_states((n_horizon_ - 1) * n_states_ + 3);
            prev_delta[n_horizon_ - 1] = min(max(pred_states((n_horizon_ - 1) * n_states_ + 4), -25.0 * M_PI / 180.0), 25.0 * M_PI / 180.0);
        }

        // Construct x_prev
        for (size_t i = 0; i < n_horizon_; ++i){
            x_prev(i * n_states_) = rotated_y;              // y
            x_prev(i * n_states_ + 1) = prev_vy[i];         // vy
            x_prev(i * n_states_ + 2) = transformed_headings[i+1];        // heading respect the reference frame
            x_prev(i * n_states_ + 3) = prev_r[i];          // yaw rate
            x_prev(i * n_states_ + 4) = prev_delta[i];      // Steering
            x_prev(i * n_states_ + 5) = delta_dot_filtered; // Filtered steering dot
        }

        Cf_ = Df * Cf * Bf;
        Cr_ = Dr * Cr * Br;

        // Contruct n_horizon_ A, B, C, D matrices
        for (size_t i = 0; i < n_horizon_; ++i){
            
            vx[i] = max(2.0, vx[i]);
            // vx[i] = max(2.0, car_state(3));

            Eigen::VectorXd prev_state = x_prev.segment(i * n_states_, n_states_);

            // Define matrix A
            A[i] << 0, cos(prev_state[2]), vx[i]*cos(prev_state[2]), 0, 0, 0,
                    0, (Cf_ * cos(prev_delta[i]) + Cr_) / (m_ * vx[i]), 0, ((lf_ * Cf_ * cos(prev_delta[i]) - lr_ * Cr_) / (m_ * vx[i])) - vx[i], -Cf_ * cos(prev_delta[i]) / m_, 0,
                    0, 0, 0, 1, 0, 0,
                    0, (lf_ * Cf_ * cos(prev_delta[i]) - lr_ * Cr_) / (Iz_ * vx[i]), 0, (lf_ * lf_ * Cf_ * cos(prev_delta[i]) + lr_ * lr_ * Cr_) / (Iz_ * vx[i]), -lf_ * Cf_ * cos(prev_delta[i]) / Iz_, 0, 
                    0, 0, 0, 0, 0, 1,
                    0, 0, 0, 0, - omega_ * omega_, - 2.0 * damp_ * omega_;

            // Define vector b
            B[i] << 0, 0, 0, 0, 0, omega_ * omega_;

            // Eigen::MatrixXd A_discrete = (A[i] * Ts_).exp(); // Matrix exponential of A * Ts_

            // Discrete-time system matrix C
            // C[i] = A_discrete;
            Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n_states_, n_states_);
            C[i] = I + A[i] * Ts_;

            // For D, you can approximate the integral with the following:
            // D[i] = (A_discrete - I) * A[i].ldlt().solve(B[i]);
            D[i] = B[i] * Ts_;
        }

        // Build S and T matrices
        C_powers[0].setIdentity();
        for (size_t i = 1; i <= n_horizon_; ++i)
            C_powers[i].noalias() = C_powers[i - 1] * C[i - 1];

        for (size_t i = 0; i < n_horizon_; ++i)
            T.block(n_states_ * i, 0, n_states_, n_states_) = C_powers[i + 1];

        S.setZero();
        
        for (size_t i = 0; i < n_horizon_; ++i){
            for (size_t j = 0; j <= i; ++j)
                S.block(n_states_ * i, j, n_states_, 1) = C_powers[i - j] * D[i - j];
        }
        
        QS = (S.array().colwise() * q_diag.array()).eval();

        S_transpose.noalias() = S.transpose();

        StQS.noalias() = S_transpose * QS;

        // Build H and g matrices
        H.noalias() = 2 * (StQS + R_);

        // Build g matrix
        g.noalias() = (2 * (x0.transpose() * T.transpose() - x_ref.transpose()) * QS).transpose();
    
    }

    // Solves the optimization problem and finds the optimal solution
    void solve(){
        PROFC_NODE_
        
        // Solution without constraints
        delta_u_opt = H.ldlt().solve(-g); // Cholesk variant (for positive and negative defined matrices)
            // delta_u_opt = H.llt().solve(-g); // Cholesky decomposition (need to find if H is positive define)

        // Solution with constraints using HPIPM solver
            // solver.solve(H, g, S, T, x0);
            // delta_u_opt = solver.getSolution();

        // Print the first 10 values of delta_u_opt
        if (verbose_)
            std::cout << "First 10 values of delta_u_opt: \n" << delta_u_opt.block(0, 0, 10, 1).transpose() << std::endl;

        // Calculate predicted states
        pred_states = S * delta_u_opt + T * x0;

        if(cfg.mpc.save_debug){
            appendSingleRowToCSV(delta_u_opt, "delta_u_opt");
            appendSingleRowToCSV(pred_states, "pred_states");
        }

        if (verbose_)
        std::cout << "First 20 values of pred_states: \n" << pred_states.block(0, 0, 20, 1).transpose() << std::endl;
        
        // Debugging steering commands
        // for (size_t i = 0; i < 5; ++i) {
            //     cout << "predicted y (" << i <<") = " << pred_states(i*n_states_) << endl;
            //     cout << "predicted vy (" << i <<") = " << pred_states(i*n_states_ + 1) << endl;
            //     cout << "predicted heading (" << i <<") = " << pred_states(i*n_states_ + 2) << endl;
            //     cout << "predicted yaw rate (" << i <<") = " << pred_states(i*n_states_ + 3) << endl;
            //     cout << "steering state (" << i <<") = " << pred_states(i*n_states_ + 4) << endl;
            //     cout << "steering dot state (" << i <<") = " << pred_states(i*n_states_ + 5) << endl;
            //     cout << "steering command (" << i <<") = " << delta_u_opt(i*n_controls_) << endl;
            // }
    }

    // Get the first steering command
    double getSteeringCmd(){
        return steeringSafety(delta_u_opt(latency_ * n_controls_));
    }

    Eigen::MatrixXd getPredictedStates(){
        Eigen::MatrixXd pred_positions_frame = Eigen::MatrixXd::Zero(n_horizon_, 4);
        
        for (size_t i = 0; i < n_horizon_; ++i){     

        double rotated_x = planner_traj_frame[i](0);  
        double rotated_y = pred_states(i * n_states_);
        double inverse_rotated_x = rotated_x * cos_ref - rotated_y * sin_ref;
        double inverse_rotated_y = rotated_x * sin_ref + rotated_y * cos_ref;
        double global_x = inverse_rotated_x + reference_x;
        double global_y = inverse_rotated_y + reference_y;
        
        pred_positions_frame(i, 0) = global_x;
        pred_positions_frame(i, 1) = global_y;
        pred_positions_frame(i, 2) = continuous(pred_states(i * n_states_ + 2) + reference_heading, 0.0);
        pred_positions_frame(i, 3) = pred_states(i * n_states_ + 4);
        }
    
        return pred_positions_frame;      
    }

    Eigen::MatrixXd getActualState(){
        Eigen::MatrixXd reference_position = Eigen::MatrixXd::Zero(n_horizon_, 2);
        
        for (size_t i = 0; i < n_horizon_; ++i){
            double rotated_x = planner_traj_frame[i](0);
            double rotated_y = planner_traj_frame[i](1);
            double inverse_rotated_x = rotated_x * cos_ref - rotated_y * sin_ref;
            double inverse_rotated_y = rotated_x * sin_ref + rotated_y * cos_ref;
            double global_x = inverse_rotated_x + reference_x;
            double global_y = inverse_rotated_y + reference_y;
            
            reference_position(i, 0) = planner_traj_frame[i](0); // global_x;
            reference_position(i, 1) = planner_traj_frame[i](1); // global_y;
        }

        return reference_position;
    }
};