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

    // Objects
    Config& cfg;
    Solver solver;
    std::unique_ptr<Model> model;

    // Constants
    const size_t n_states = 6;
    const size_t n_controls = 2;
    size_t n_horizon;
    const float Ts = 0.025;

    // Weights matrices
    Eigen::VectorXd q_diag;
    Eigen::MatrixXd R, Rd, R_, Rd_, D;

    // Model matrices
    ModelMatrices mpc_matrices;
    ModelMatrices evaluator_matrices;

    // Solve matrices
    Eigen::MatrixXd H;
    Eigen::VectorXd g, u_opt, d;

    // Flags
    bool firstIteration = true;

    /////////////////////////////////////////////////////////////////////////
    //-------------------------- MPC functions  ---------------------------//

    // Builds weights matrices
    void createWeights()
    {
        PROFC_NODE_

        // Q_ matrix diagonal
        Eigen::VectorXd diag_values(n_states);
        diag_values << cfg.mpc.q_lat / pow(cfg.mpc.scale_y, 2),
                       cfg.mpc.q_vy / pow(cfg.mpc.scale_vy, 2), 
                       cfg.mpc.q_phi / pow(cfg.mpc.scale_vy, 2), 
                       cfg.mpc.q_r / pow(cfg.mpc.scale_r, 2), 
                       cfg.mpc.q_delta / pow(cfg.mpc.scale_st, 2), 
                       cfg.mpc.q_delta_dot / pow(cfg.mpc.scale_dst, 2);

        for (size_t i = 0; i < (n_horizon-1); ++i)
            q_diag.segment(i * n_states, n_states) = diag_values;

        q_diag.segment((n_horizon-1)*n_states, n_states) << 
                cfg.mpc.p_lat / pow(cfg.mpc.scale_y, 2), 
                cfg.mpc.p_vy / pow(cfg.mpc.scale_vy, 2), 
                cfg.mpc.p_phi / pow(cfg.mpc.scale_phi, 2), 
                cfg.mpc.p_r / pow(cfg.mpc.scale_r, 2), 
                cfg.mpc.p_delta / pow(cfg.mpc.scale_st, 2), 
                cfg.mpc.p_delta_dot / pow(cfg.mpc.scale_dst, 2);

        // Create matrix R_
        R.setZero();
        R(0, 0) = cfg.mpc.r_st / pow(cfg.mpc.scale_st, 2);
        R(1, 1) = cfg.mpc.r_mz / pow(cfg.mpc.scale_mz, 2);
        R_.setZero(); 
        for (size_t i = 0; i < n_horizon; ++i)
            R_.block(i * n_controls, i * n_controls, n_controls, n_controls) = R;

        // Create matrix Rd_
        Rd.setZero();
        Rd(0, 0) = cfg.mpc.rd_st / pow(cfg.mpc.scale_dst, 2);
        Rd(1, 1) = cfg.mpc.rd_mz / pow(cfg.mpc.scale_dmz, 2);
        Rd_.setZero();
        for (size_t i = 0; i < n_horizon; ++i)
            Rd_.block(i * n_controls, i * n_controls, n_controls, n_controls) = Rd;
        
        // Create matrix D
        D.setZero();
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n_controls, n_controls);
        for (size_t k = 0; k < n_horizon; ++k)
        {
            const size_t diag = k * n_controls;

            // Diagonal block
            D.block(diag, diag, n_controls, n_controls) = I;

            // Subdiagonal block
            if (k > 0)
            {
                const size_t col_prev = (k - 1) * n_controls;
                D.block(diag, col_prev, n_controls, n_controls) = -I;
            }
        }

        if(cfg.save_debug){
            appendSingleRowToCSV(q_diag, "q_diag");
            appendSingleRowToCSV(R_, "R_");
            appendSingleRowToCSV(Rd_, "Rd_");
            appendSingleRowToCSV(D, "D_");
        }
    }

    // Builds x_ref vector
    void createReference(const States &local_ref)
    {
        PROFC_NODE_

        for (size_t i = 0; i < n_horizon; ++i){
            if (i < local_ref.size()){
                mpc_matrices.x_ref.segment(i * n_states, n_states) << local_ref[i].y,           // y
                                                                        local_ref[i].vy,        // vy
                                                                        local_ref[i].psi,       // phi
                                                                        local_ref[i].r,         // r
                                                                        local_ref[i].delta,     // delta
                                                                        local_ref[i].delta_dot; // delta dot
            }else{
                mpc_matrices.x_ref.segment(i * n_states, n_states) = mpc_matrices.x_ref.segment((i - 1) * n_states, n_states);
            }
        }

        if(cfg.verbose){
            std::cout << "Reference trajectory (first 5 points): " << std::endl;
            for (size_t i = 0; i < std::min(size_t(5), local_ref.size()); ++i){
                std::cout << "Point " << i << ": y: " << local_ref[i].y << ", vy: " << local_ref[i].vy << ", psi: " << local_ref[i].psi << ", r: " << local_ref[i].r << ", delta: " << local_ref[i].delta << ", delta_dot: " << local_ref[i].delta_dot << std::endl;
            }
        }

        if(cfg.save_debug){
            appendSingleRowToCSV(mpc_matrices.x_ref, "x_ref");
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
        for (size_t i = 0; i < n_horizon; ++i){
            if (i < prev_states.size()){
                m.x_prev.segment(i * n_states, n_states) << prev_states[i].y,       // y
                                                            prev_states[i].vy,        // vy
                                                            prev_states[i].psi,       // phi
                                                            prev_states[i].r,         // r
                                                            prev_states[i].delta,     // delta
                                                            prev_states[i].delta_dot; // delta dot
            }else{
                m.x_prev.segment(i * n_states, n_states) = m.x_prev.segment((i - 1) * n_states, n_states);
            }
        }

        // vx vector
        for (size_t i = 0; i < n_horizon; ++i){
            if (i < prev_states.size()){
                m.vx[i] = prev_states[i].vx;
            }else{
                m.vx[i] = m.vx[i - 1];
            }
        }

        // u_prev vector
        for (size_t i = 0; i < n_horizon; ++i){
            if (i < prev_controls.size()){
                m.u_prev(i * n_controls) = prev_controls[i].steering; // steering
                m.u_prev(i * n_controls +1) = prev_controls[i].mz; // steering
            }else{
                m.u_prev.segment(i * n_controls, n_controls) = m.u_prev.segment((i - 1) * n_controls, n_controls);
            }
        }

        // Discrete model matrices Ad Bd Cd
        for (size_t i = 0; i < n_horizon; ++i){
            auto prev_state = m.x_prev.segment(i * n_states, n_states);
            auto prev_u = m.u_prev.segment(i * n_controls, n_controls);

            model->getDiscreteMatrices(prev_state, prev_u, m.vx[i], m.Ad[i], m.Bd[i], m.Cd[i]);

            std::cout << "Ad[" << i << "]:\n" << m.Ad[i] << std::endl;
            std::cout << "Bd[" << i << "]:\n" << m.Bd[i] << std::endl;
            std::cout << "Cd[" << i << "]:\n" << m.Cd[i].transpose() << std::endl;

            // Sanity check
            if(!m.Ad[i].allFinite() || !m.Bd[i].allFinite() || !m.Cd[i].allFinite()){
                std::cout << "NaN in model matrices at step " << i << std::endl;
            }
        }


        // Fill matrix T // TODO: optimize
        m.T.setZero();
        for (int i = 0; i < n_horizon; ++i)
        {
            m.Aprod.setIdentity();
            for (int k = 0; k <= i; ++k)
            {
                m.Aprod = m.Ad[k] * m.Aprod;
            }

            m.T.block(i * n_states, 0, n_states, n_states) = m.Aprod;
        }

        // Fill matrix S
        m.S.setZero();
        for (int i = 0; i < n_horizon; ++i)
        {
            for (int j = 0; j <= i; ++j)
            {
                m.Aprod.setIdentity();

                for (int k = j + 1; k <= i; ++k)
                {
                    m.Aprod = m.Ad[k] * m.Aprod;
                }

                m.S.block(i * n_states, j * n_controls, n_states, n_controls) = m.Aprod * m.Bd[j];
            }
        }

        // Fill matrix W
        m.W.setZero();
        for (int i = 0; i < n_horizon; ++i)
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

            m.W.segment(i * n_states, n_states) = m.wk;
        }

        if(cfg.verbose){
            std::cout << "Size of x0: " << m.x0.size() << std::endl;
            std::cout << "Size of x_ref: " << m.x_ref.size() << std::endl;
            std::cout << "Size of x_prev: " << m.x_prev.size() << std::endl;
            std::cout << "Size of u_prev: " << m.u_prev.size() << std::endl;
            std::cout << "Size of Ad: " << m.Ad.size() << ", each of size: " << m.Ad[0].rows() << "x" << m.Ad[0].cols() << std::endl;
            std::cout << "Size of Bd: " << m.Bd.size() << ", each of size: " << m.Bd[0].rows() << "x" << m.Bd[0].cols() << std::endl;
            std::cout << "Size of Cd: " << m.Cd.size() << ", each of size: " << m.Cd[0].size() << std::endl;
            std::cout << "Size of T: " << m.T.rows() << "x" << m.T.cols() << std::endl;
            std::cout << "Size of S: " << m.S.rows() << "x" << m.S.cols() << std::endl;
            std::cout << "Size of W: " << m.W.size() << std::endl;
        }

        if(cfg.save_debug){
            appendSingleRowToCSV(m.x0, "x0");
            appendSingleRowToCSV(m.x_prev, "x_prev");
            appendSingleRowToCSV(m.vx, "vx");
            appendSingleRowToCSV(m.u_prev, "u_prev");
            appendSingleRowToCSV(m.T, "T");
            appendSingleRowToCSV(m.S, "S");
            appendSingleRowToCSV(m.W, "W");
        }
    }

    // Solve the optimisation problem
    Controls solve(ModelMatrices &m, Control u_prev_iter)
    {
        PROFC_NODE_

        std::cout << "q_diag min/max: "
          << q_diag.minCoeff() << " / " << q_diag.maxCoeff() << std::endl;

        std::cout << "R_ min/max: "
                << R_.minCoeff() << " / " << R_.maxCoeff() << std::endl;

        std::cout << "S min/max: "
                << m.S.minCoeff() << " / " << m.S.maxCoeff() << std::endl;

        std::cout << "T min/max: "
                << m.T.minCoeff() << " / " << m.T.maxCoeff() << std::endl;

        std::cout << "W min/max: "
                << m.W.minCoeff() << " / " << m.W.maxCoeff() << std::endl;

        std::cout << "x0: " << m.x0.transpose() << std::endl;
        std::cout << "u_prev: " << m.u_prev.transpose() << std::endl;

        Eigen::DiagonalMatrix<double, Eigen::Dynamic> Q(q_diag);

        d.setZero();
        d(0) = -u_prev_iter.steering;
        d(1) = -u_prev_iter.mz;

        
        H = 2.0 * (m.S.transpose() * Q * m.S + R_ + D.transpose() * Rd_ * D);
        H = 0.5 * (H + H.transpose());
        H.diagonal().array() += 1e-8;
        
        Eigen::VectorXd e = m.T * m.x0 + m.W - m.x_ref;
        g = 2.0 * (m.S.transpose() * Q * e) + 2.0 * D.transpose() * Rd_ * d; // TODO: 2 is correct?

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

        Controls optimal_controls(n_horizon);
        for (size_t i = 0; i < n_horizon; ++i){
            optimal_controls[i].steering = u_opt(i * n_controls);
        }

        if(cfg.verbose){
            std::cout << "Optimal control sequence (first 5 points): " << std::endl;
            for (size_t i = 0; i < std::min(size_t(5), optimal_controls.size()); ++i){
                std::cout << "Control " << i << ": steering: " << optimal_controls[i].steering << std::endl;
            }
        }

        if(cfg.save_debug){
            appendSingleRowToCSV(d, "d");
            appendSingleRowToCSV(H, "H");
            appendSingleRowToCSV(g, "g");
            appendSingleRowToCSV(u_opt, "u_opt");
        }

        return optimal_controls;
    }

    // Predict the following states given initial state and controls
    States predict_states(const State &x_0, const Controls &controls, const ModelMatrices &m){
        PROFC_NODE_

        States predicted_states(n_horizon);

        Eigen::VectorXd x_pred(n_states*n_horizon);
        Eigen::VectorXd u_vec(n_controls*n_horizon);
        Eigen::VectorXd x0_vec(n_states);

        x0_vec << x_0.y, x_0.vy, x_0.psi, x_0.r, x_0.delta, x_0.delta_dot;
        u_vec.setZero();
        for (size_t i = 0; i < controls.size(); ++i){
            u_vec(i * n_controls) = controls[i].steering;
        }

        x_pred.noalias() = m.T * x0_vec + m.S * u_vec + m.W;

        for (size_t i = 0; i < n_horizon; ++i){
            predicted_states[i].y = x_pred(i * n_states);
            predicted_states[i].vy = x_pred(i * n_states + 1);
            predicted_states[i].psi = x_pred(i * n_states + 2);
            predicted_states[i].r = x_pred(i * n_states + 3);
            predicted_states[i].delta = x_pred(i * n_states + 4);
            predicted_states[i].delta_dot = x_pred(i * n_states + 5);
        }

        if(cfg.save_debug){
            appendSingleRowToCSV(x_pred, "x_pred");
        }

        return predicted_states;
    }

    /////////////////////////////////////////////////////////////////////////
    //-------------------------- Auxiliar functions  ----------------------//

    // Initialize size of the model matrices
    void initModelMatrices(ModelMatrices& m)
    {
        m.x0.resize(n_states);
        m.x_ref.resize(n_states * n_horizon);
        m.x_prev.resize(n_states * n_horizon);
        m.u_prev.resize(n_controls * n_horizon);
        m.vx.resize(n_horizon);

        m.Ad.resize(n_horizon);
        m.Bd.resize(n_horizon);
        m.Cd.resize(n_horizon);

        for (size_t i = 0; i < n_horizon; ++i){
            m.Ad[i].resize(n_states, n_states);
            m.Bd[i].resize(n_states, n_controls);
            m.Cd[i].resize(n_states);
        }

        m.Aprod.resize(n_states, n_states);
        m.wk.resize(n_states);

        m.S.resize(n_horizon * n_states, n_horizon * n_controls);
        m.T.resize(n_horizon * n_states, n_states);
        m.W.resize(n_horizon * n_states);
    }
    
    // Safety function for steering command
    double steeringSafety(const double &steering, const State &car_state){
        if (std::isnan(steering))
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

    // Debugging function to append a single row to a CSV file (vector of doubles)
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

    // Debugging function to append a single row to a CSV file (Eigen vector)
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

    // Debugging function to append a single row to a CSV file (matrix)
    void appendSingleRowToCSV(const Eigen::MatrixXd& matrix, const std::string& vectorname){
        const std::string filename = cfg.ws_path + cfg.debug_path + "/" + vectorname + ".csv";
        std::ofstream file(filename, std::ios::app);

        if (!file.is_open())
        {
            std::cerr << "Error: Could not open csv for writing: " << filename << std::endl;
            return;
        }

        // Flatten matrix as a single CSV row in row-major order
        for (Eigen::Index r = 0; r < matrix.rows(); ++r) {
            for (Eigen::Index c = 0; c < matrix.cols(); ++c) {
                file << matrix(r, c);

                if (!(r == matrix.rows() - 1 && c == matrix.cols() - 1))
                    file << ",";
            }
        }
        file << "\n";

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
        n_horizon = cfg.mpc.n_horizon;

        // Initialize model matrices
        initModelMatrices(mpc_matrices);
        initModelMatrices(evaluator_matrices);

        // Initialize weights matrices
        q_diag.resize(n_states * n_horizon);
        R.resize(n_controls, n_controls);
        Rd.resize(n_controls, n_controls);
        R_.resize(n_horizon * n_controls, n_horizon * n_controls);
        Rd_.resize(n_horizon * n_controls, n_horizon * n_controls);
        D.resize(n_horizon * n_controls, n_horizon * n_controls);

        // Initialize solver matrices
        H.resize(n_horizon * n_controls, n_horizon * n_controls);
        g.resize(n_horizon * n_controls);
        u_opt.resize(n_horizon * n_controls);
        d.resize(n_horizon * n_controls);

        createWeights();

        firstIteration = true;

        // TODO: This as a parameter
        double max_steering = 25.0 * M_PI / 180.0; // rad
        double max_steering_dot = 80.0 * M_PI / 180.0; // rad/s

        solver.setParams(max_steering, max_steering_dot, n_states, n_horizon, n_controls, cfg.verbose);
        std::cout << "MPC initialized" << std::endl;
    }

    ~MPC() = default;

    void compute_mpc(const State &car_state, const Control u_prev_iter, const States &local_ref, const States &x_prev, const Controls &u_prev, States &predicted_states, Controls &optimal_controls)
    {
        PROFC_NODE_

        if (cfg.verbose){
            std::cout << "Current state: y: " << car_state.y << ", vy: " << car_state.vy << ", psi: " << car_state.psi << ", r: " << car_state.r << ", delta: " << car_state.delta << ", delta_dot: " << car_state.delta_dot << std::endl;
            std::cout << "Size of local reference: " << local_ref.size() << std::endl;
            std::cout << "Size of previous states: " << x_prev.size() << std::endl;
            std::cout << "Size of previous controls: " << u_prev.size() << std::endl;
        }

        createReference(local_ref);
        createModelMatrices(car_state, x_prev, u_prev, mpc_matrices);
        optimal_controls = solve(mpc_matrices, u_prev_iter);
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
        createWeights();
    }
};