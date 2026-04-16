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
 
#include "utils/solver.hpp"
#include "utils/Config.hpp"
#include <as_lib/utils/Profiler.hpp>

#include "models/ltv_model.hpp"
#include "models/anfis_model.hpp"

class Point : public std::array<double, 2> {
    public:
    static const int DIM = 2;
};

struct ModelMatrices{

    // For inputs
    Eigen::VectorXd x0;
    Eigen::VectorXd x_ref;
    Eigen::VectorXd x_prev;
    Eigen::VectorXd u_prev;
    std::vector<double> vx;

    // For discrete models
    std::vector<Eigen::MatrixXd> Ad;
    std::vector<Eigen::MatrixXd> Bd;
    std::vector<Eigen::VectorXd> Cd;

    // Intermediate calculations
    Eigen::MatrixXd Aprod;
    Eigen::VectorXd wk;

    // For prediction
    Eigen::MatrixXd S;
    Eigen::MatrixXd T;
    Eigen::VectorXd W;
};

using States = std::vector<State>;
using Controls = std::vector<Control>;

class MPC {
       
  private:
    Config& cfg;
    Solver solver;
    std::unique_ptr<Model> model;

    // Model matrices
    ModelMatrices mpc_matrices;
    ModelMatrices evaluator_matrices;

    Eigen::MatrixXd QS, S_transpose, StQS, H;
    Eigen::VectorXd g, u_opt;

    // Weights matrices
    Eigen::VectorXd q_diag;
    Eigen::MatrixXd R;
    Eigen::MatrixXd R_;

    // Variables
    int n_states_, n_controls_, n_horizon_, n_planning_;
    double Ts_, disc_;
    bool verbose_;

    // Flags
    bool firstIteration = true;
    bool init_state = true;

    /////////////////////////////////////////////////////////////////////////
    //-------------------------- MPC functions  ---------------------------//

    // Builds weights matrices
    void createWeights(bool init_state)
    {
        PROFC_NODE_
        
        // Create matrix R
        R.setZero();
        if (init_state)
            R(0, 0) = cfg.mpc.r_st_init;
        else
            R(0, 0) = cfg.mpc.r_st;

        // Q_ matrix diagonal
        Eigen::VectorXd diag_values(n_states_);
        if (init_state)
            diag_values << cfg.mpc.q_lat_init, cfg.mpc.q_vy_init, cfg.mpc.q_phi_init, cfg.mpc.q_r, cfg.mpc.q_delta_init, cfg.mpc.q_delta_dot_init;
        else
            diag_values << cfg.mpc.q_lat, cfg.mpc.q_vy, cfg.mpc.q_phi, cfg.mpc.q_r, cfg.mpc.q_delta, cfg.mpc.q_delta_dot;

        for (size_t i = 0; i < (n_horizon_-1); ++i)
            q_diag.segment(i * n_states_, n_states_) = diag_values;

        // Last point weights
        q_diag.segment((n_horizon_-1)*n_states_, n_states_) << cfg.mpc.p_lat, cfg.mpc.p_vy, cfg.mpc.p_phi, cfg.mpc.p_r, cfg.mpc.p_delta, cfg.mpc.p_delta_dot;

        // Create matrix R_
        R_.setZero(); 
        for (size_t i = 0; i < n_horizon_; ++i)
            R_.block(i * n_controls_, i * n_controls_, n_controls_, n_controls_) = R;

        // Print the first 20x20 values of R_
        if (verbose_){
            std::cout << "diag_values: \n" << diag_values.transpose() << std::endl;
            std::cout << "R matrix: \n" << R << std::endl;
            std::cout << "First 20x20 values of R_: \n" << R_.block(0, 0, 20, 20) << std::endl;
        }
    }

    // Builds the matrixes x_0, x_prev, u_prev, T, S and W. Saves them to Model Matrixes.
    void createModelMatrices(const State &car_state, const States &prev_states, const Controls &prev_controls, ModelMatrices &m)
    {
        PROFC_NODE_

        // x_0 vector
        m.x0 << car_state.y,        // y
              car_state.vy,         // vy
              car_state.psi,        // phi
              car_state.r,          // r
              car_state.delta,      // delta
              car_state.delta_dot;  // delta dot

        // x_prev vector
        for (size_t i = 0; i < n_horizon_; ++i){
            if (i < prev_states.size()){
                m.x_prev.segment(i * n_states_, n_states_) << prev_states[i].y,       // y
                                                            prev_states[i].vy,        // vy
                                                            prev_states[i].psi,       // phi
                                                            prev_states[i].r,         // r
                                                            prev_states[i].delta,     // delta
                                                            prev_states[i].delta_dot; // delta dot
            }else{
                m.x_prev.segment(i * n_states_, n_states_) = m.x_prev.segment((i - 1) * n_states_, n_states_);
            }
        }

        // vx vector
        for (size_t i = 0; i <= n_horizon_; ++i){
            if (i < prev_states.size()){
                m.vx[i] = prev_states[i].vx;
            }else{
                m.vx[i] = m.vx[i - 1];
            }
        }

        // u_prev vector
        for (size_t i = 0; i < n_horizon_; ++i){
            if (i < prev_controls.size()){
                m.u_prev.segment(i * n_controls_, n_controls_) << prev_controls[i].steering; // steering
            }else{
                m.u_prev.segment(i * n_controls_, n_controls_) = m.u_prev.segment((i - 1) * n_controls_, n_controls_);
            }
        }

        // Discrete model matrices Ad Bd Cd
        for (size_t i = 0; i < n_horizon_; ++i){
            auto prev_state = m.x_prev.segment(i * n_states_, n_states_);
            auto prev_u = m.u_prev.segment(i * n_controls_, n_controls_);

            model->getDiscreteMatrices(prev_state, prev_u, m.vx[i], m.Ad[i], m.Bd[i], m.Cd[i]);
            
            // Sanity check
            if(!m.Ad[i].allFinite() || !m.Bd[i].allFinite() || !m.Cd[i].allFinite()){
                std::cout << "NaN in model matrices at step " << i << std::endl;
            }
        }


        // Fill matrix T
        m.T.setZero();
        for (int i = 0; i < n_horizon_; ++i)
        {
            m.Aprod.setIdentity();

            for (int k = 0; k <= i; ++k)
            {
                m.Aprod = m.Ad[k] * m.Aprod;
            }

            m.T.block(i * n_states_, 0, n_states_, n_states_) = m.Aprod;
        }

        // Fill matrix S
        m.S.setZero();
        for (int i = 0; i < n_horizon_; ++i)
        {
            for (int j = 0; j <= i; ++j)
            {
                m.Aprod.setIdentity();

                for (int k = j + 1; k <= i; ++k)
                {
                    m.Aprod = m.Ad[k] * m.Aprod;
                }

                m.S.block(i * n_states_, j * n_controls_, n_states_, n_controls_) = m.Aprod * m.Bd[j];
            }
        }

        // Fill matrix W
        m.W.setZero();
        for (int i = 0; i < n_horizon_; ++i)
        {
            m.wk.setZero();

            for (int j = 0; j <= i; ++j)
            {
                m.Aprod.setIdentity();

                for (int k = j + 1; k <= i; ++k)
                {
                    m.Aprod = m.Ad[k] * m.Aprod;
                }

                m.wk.noalias() += m.Aprod * m.Cd[j];
            }

            m.W.segment(i * n_states_, n_states_) = m.wk;
        }

        if(verbose_){
            std::cout << "Size of x0: " << m.x0.size() << std::endl;
            std::cout << "Size of x_ref: " << m.x_ref.size() << std::endl;
            std::cout << "Size of x_prev: " << m.x_prev.size() << std::endl;
            std::cout << "Size of u_prev: " << m.u_prev.size() << std::endl;
            std::cout << "Size of Ad: " << m.Ad.size() << ", each of size: " << m.Ad[0].rows() << "x" << m.Ad[0].cols() << std::endl;
            std::cout << "Size of Bd: " << m.Bd.size() << ", each of size: " << m.Bd[0].rows() << "x" << m.Bd[0].cols() << std::endl;
            std::cout << "Size of Cd: " << m.Cd.size() << ", each of size: " << m.Cd[0].size() << std::endl;
            std::cout << "Size of T: " << m.T.rows() << "x" << m.T.cols() << std::endl;
            std::cout << "Size of S: " << m.S.rows() << "x" << m.S.cols() << std::endl;
        }
    }

    // Builds x_ref vector
    void createReference(const State &car_state, const States &local_ref)
    {
        PROFC_NODE_

        for (size_t i = 0; i < n_horizon_; ++i){
            if (i < local_ref.size()){
                mpc_matrices.x_ref.segment(i * n_states_, n_states_) << local_ref[i].y,         // y
                                                                        local_ref[i].vy,        // vy
                                                                        local_ref[i].psi,       // phi
                                                                        local_ref[i].r,         // r
                                                                        local_ref[i].delta,     // delta
                                                                        local_ref[i].delta_dot; // delta dot
            }else{
                mpc_matrices.x_ref.segment(i * n_states_, n_states_) = mpc_matrices.x_ref.segment((i - 1) * n_states_, n_states_);
            }
        }

        if(verbose_){
            std::cout << "Reference trajectory (first 5 points): " << std::endl;
            for (size_t i = 0; i < std::min(size_t(5), local_ref.size()); ++i){
                std::cout << "Point " << i << ": y: " << local_ref[i].y << ", vy: " << local_ref[i].vy << ", psi: " << local_ref[i].psi << ", r: " << local_ref[i].r << ", delta: " << local_ref[i].delta << ", delta_dot: " << local_ref[i].delta_dot << std::endl;
            }
        }
    }

    // Solve the optimisation problem
    Controls solve(ModelMatrices &m)
    {
        PROFC_NODE_

        QS = (m.S.array().colwise() * q_diag.array()).eval();
        S_transpose.noalias() = m.S.transpose();
        StQS.noalias() = S_transpose * QS;

        H.noalias() = 2 * (StQS + R_);
        Eigen::VectorXd e = m.T * m.x0 + m.W - m.x_ref;
        g.noalias() = 2.0 * m.S.transpose() * e.cwiseProduct(q_diag);   

        // Sanity check
        if(!H.allFinite()){
            std::cerr << "Error: H matrix contains non-finite values" << std::endl;
        }
        if(!g.allFinite()){
            std::cerr << "Error: g vector contains non-finite values" << std::endl;
        }
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(H);
        double min_eig = eig.eigenvalues().minCoeff();
        if (min_eig <= 0){
            std::cerr << "Error: H matrix is not positive definite, min eigenvalue: " << min_eig << std::endl;
        }

        // Solution without constraints
        u_opt = H.ldlt().solve(-g); // Cholesk variant (for positive and negative defined matrices)
            // u_opt = H.llt().solve(-g); // Cholesky decomposition (need to find if H is positive define)

        // Solution with constraints using HPIPM solver
            // solver.solve(H, g, S, T, x0);
            // u_opt = solver.getSolution();

        Controls optimal_controls(n_horizon_);
        for (size_t i = 0; i < n_horizon_; ++i){
            optimal_controls[i].steering = u_opt(i * n_controls_);
        }

        if(verbose_){
            std::cout << "Optimal control sequence (first 5 points): " << std::endl;
            for (size_t i = 0; i < std::min(size_t(5), optimal_controls.size()); ++i){
                std::cout << "Control " << i << ": steering: " << optimal_controls[i].steering << std::endl;
            }
        }

        return optimal_controls;
    }

    // Predict the following states given initial state and controls
    States predict_states(const State &x_0, const Controls &controls, const ModelMatrices &m){
        PROFC_NODE_

        States predicted_states(n_horizon_);

        Eigen::VectorXd x_pred(n_states_*n_horizon_);
        Eigen::VectorXd u_vec(n_controls_*n_horizon_);
        Eigen::VectorXd x0_vec(n_states_);

        x0_vec << x_0.y, x_0.vy, x_0.psi, x_0.r, x_0.delta, x_0.delta_dot;
        u_vec.setZero();
        for (size_t i = 0; i < controls.size(); ++i){
            u_vec(i * n_controls_) = controls[i].steering;
        }

        x_pred.noalias() = m.T * x0_vec + m.S * u_vec + m.W;

        for (size_t i = 0; i < n_horizon_; ++i){
            predicted_states[i].y = x_pred(i * n_states_);
            predicted_states[i].vy = x_pred(i * n_states_ + 1);
            predicted_states[i].psi = x_pred(i * n_states_ + 2);
            predicted_states[i].r = x_pred(i * n_states_ + 3);
            predicted_states[i].delta = x_pred(i * n_states_ + 4);
            predicted_states[i].delta_dot = x_pred(i * n_states_ + 5);
        }

        return predicted_states;
    }

    /////////////////////////////////////////////////////////////////////////
    //-------------------------- Auxiliar functions  ----------------------//

    // Initialize size of the model matrices
    void initModelMatrices(ModelMatrices& m)
    {
        m.x0.resize(n_states_);
        m.x_ref.resize(n_states_ * n_horizon_);
        m.x_prev.resize(n_states_ * n_horizon_);
        m.u_prev.resize(n_controls_ * n_horizon_);
        m.vx.resize(n_horizon_ + 1);

        m.Ad.resize(n_horizon_);
        m.Bd.resize(n_horizon_);
        m.Cd.resize(n_horizon_);

        for (size_t i = 0; i < n_horizon_; ++i){
            m.Ad[i].resize(n_states_, n_states_);
            m.Bd[i].resize(n_states_, n_controls_);
            m.Cd[i].resize(n_states_);
        }

        m.Aprod.resize(n_states_, n_states_);
        m.wk.resize(n_states_);

        m.S.resize(n_horizon_ * n_states_, n_horizon_ * n_controls_);
        m.T.resize(n_horizon_ * n_states_, n_states_);
        m.W.resize(n_horizon_ * n_states_);
    }
    
    // Safety function for steering command
    double steeringSafety(const double &steering, const State &car_state){
        if (isnan(steering))
            return 0.0;
        else if (car_state.vx < 0.3)
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
        std::string filename = cfg.ws_path + cfg.debug_path + "/" + vectorname + ".csv";

        // Open the CSV file in append mode
        std::ofstream file(filename, std::ios::app);

        if (!file.is_open())
        {
            std::cerr << "Error: Could not open csv for writing: " << filename << std::endl;
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
        std::string filename = cfg.ws_path + cfg.debug_path + "/" + vectorname + ".csv";        
        // Open the CSV file in append mode
        std::ofstream file(filename, std::ios::app);
        
        if (!file.is_open())
        {
            std::cerr << "Error: Could not open csv for writing: " << filename << std::endl;
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

        // Model
        model = std::make_unique<LtvModel>();
        model->initialize();

        // Save recurrent parameters
        n_states_ = cfg.mpc.n_states;
        n_controls_ = cfg.mpc.n_controls;
        n_horizon_ = cfg.mpc.n_horizon;
        n_planning_ = cfg.mpc.n_planning;
        Ts_ = cfg.mpc.Ts;
        disc_ = cfg.mpc.disc;
        verbose_ = cfg.mpc.verbose;

        // Initialize model matrices
        initModelMatrices(mpc_matrices);
        initModelMatrices(evaluator_matrices);

        // Initialize weights matrices
        q_diag.resize(n_states_ * n_horizon_);
        R.resize(n_controls_, n_controls_);
        R_.resize(n_horizon_ * n_controls_, n_horizon_ * n_controls_);

        createWeights(true);

        firstIteration = true;

        // TODO: This as a parameter
        double max_steering = 30.0 * M_PI / 180.0; // rad
        double max_steering_dot = 80.0 * M_PI / 180.0; // rad/s

        solver.setParams(max_steering, max_steering_dot, n_states_, n_horizon_, n_controls_, verbose_);
        std::cout << "MPC initialized" << std::endl;
    }

    ~MPC() = default;

    void compute_mpc(const State &car_state, const States &local_ref, const States &x_prev, const Controls &u_prev, States &predicted_states, Controls &optimal_controls)
    {
        PROFC_NODE_

        if (verbose_){
            std::cout << "Current state: y: " << car_state.y << ", vy: " << car_state.vy << ", psi: " << car_state.psi << ", r: " << car_state.r << ", delta: " << car_state.delta << ", delta_dot: " << car_state.delta_dot << std::endl;
            std::cout << "Size of local reference: " << local_ref.size() << std::endl;
            std::cout << "Size of previous states: " << x_prev.size() << std::endl;
            std::cout << "Size of previous controls: " << u_prev.size() << std::endl;
        }

        // Check if initial state is over
        if(init_state && car_state.vx > cfg.mpc.vx_to_finish_init){
            init_state = false;
            createWeights(false);
        }

        createReference(car_state, local_ref);
        createModelMatrices(car_state, x_prev, u_prev, mpc_matrices);
        optimal_controls = solve(mpc_matrices);
        predicted_states = predict_states(car_state, optimal_controls, mpc_matrices);

        // Ensure safety
        for (size_t i = 0; i < optimal_controls.size(); ++i){
            optimal_controls[i].steering = steeringSafety(optimal_controls[i].steering, car_state);
        }
        
    }

    States compute_prediction(const State &x_0, const Controls controls, const States &x_prev, const Controls &u_prev)
    {
        PROFC_NODE_

        createModelMatrices(x_0, x_prev, u_prev, evaluator_matrices);
        States prediction = predict_states(x_0, controls, evaluator_matrices);

        return prediction;
    }

    void update_config()
    {
        createWeights(init_state);
    }
};