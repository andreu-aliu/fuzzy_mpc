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
#include "utils/kdtree.hpp"
#include <as_lib/utils/Profiler.hpp>

#include "models/ltv_model.hpp"
#include "models/anfis_model.hpp"

class Point : public std::array<double, 2> {
    public:
    static const int DIM = 2;
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

    // Weight matrices
    Eigen::VectorXd q_diag;
    Eigen::MatrixXd R, Rd, R_, Rd_, D;

    // Inputs
    Eigen::VectorXd x0;
    Eigen::VectorXd x_ref;
    Eigen::VectorXd x_prev;
    Eigen::VectorXd u_prev;
    std::vector<double> vx;

    // Discrete models
    std::vector<Eigen::MatrixXd> Ad;
    std::vector<Eigen::MatrixXd> Bd;
    std::vector<Eigen::VectorXd> Cd;

    // Intermediate calculations
    Eigen::MatrixXd Aprod;
    Eigen::MatrixXd temp6x6;
    Eigen::VectorXd wk;

    // Condensed model
    Eigen::MatrixXd S;
    Eigen::MatrixXd T;
    Eigen::VectorXd W;

    // Solve matrices
    Eigen::MatrixXd H;
    Eigen::VectorXd g, u_opt, d;

    // Prediction
    Eigen::VectorXd x_pred;
    Eigen::VectorXd u_vec;

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
                       cfg.mpc.q_phi / pow(cfg.mpc.scale_psi, 2), 
                       cfg.mpc.q_r / pow(cfg.mpc.scale_r, 2), 
                       cfg.mpc.q_delta / pow(cfg.mpc.scale_st, 2), 
                       cfg.mpc.q_delta_dot / pow(cfg.mpc.scale_dst, 2);

        for (size_t i = 0; i < (n_horizon-1); ++i)
            q_diag.segment(i * n_states, n_states) = diag_values;

        q_diag.segment((n_horizon-1)*n_states, n_states) << 
                cfg.mpc.p_lat / pow(cfg.mpc.scale_y, 2), 
                cfg.mpc.p_vy / pow(cfg.mpc.scale_vy, 2), 
                cfg.mpc.p_phi / pow(cfg.mpc.scale_psi, 2), 
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
                x_ref.segment(i * n_states, n_states) << local_ref[i].y,         // y
                                                                      local_ref[i].vy,        // vy
                                                                      local_ref[i].psi,       // phi
                                                                      local_ref[i].r,         // r
                                                                      local_ref[i].delta,     // delta
                                                                      local_ref[i].delta_dot; // delta dot
                vx[i] = local_ref[i].vx;
            }else{
                x_ref.segment(i * n_states, n_states) = x_ref.segment((i - 1) * n_states, n_states);
                vx[i] = vx[i-1];
            }
        }

        if(cfg.verbose){
            std::cout << "Reference trajectory (first 5 points): " << std::endl;
            for (size_t i = 0; i < std::min(size_t(5), local_ref.size()); ++i){
                std::cout << "Point " << i << ": y: " << local_ref[i].y << ", vy: " << local_ref[i].vy << ", psi: " << local_ref[i].psi << ", r: " << local_ref[i].r << ", delta: " << local_ref[i].delta << ", delta_dot: " << local_ref[i].delta_dot << std::endl;
            }
        }
    }
    
    // Builds the matrixes x_0, x_prev, u_prev, T, S and W. Saves them to Model Matrixes.
    void createModelMatrices(const State &car_state, const States &prev_states, const Controls &prev_controls)
    {
        PROFC_NODE_

        // x0 vector
        x0 << car_state.y,          // y
              car_state.vy,         // vy
              car_state.psi,        // phi
              car_state.r,          // r
              car_state.delta,      // delta
              car_state.delta_dot;  // delta dot

        {PROFC_NODE("createModelMatrices_first")
        for (size_t i = 0; i < n_horizon; ++i){

            // x_prev vector
            if (i < prev_states.size()-1){
                x_prev.segment(i * n_states, n_states) << prev_states[i+1].y,         // y
                                                          prev_states[i+1].vy,        // vy
                                                          prev_states[i+1].psi,       // psi
                                                          prev_states[i+1].r,         // r
                                                          prev_states[i+1].delta,     // delta
                                                          prev_states[i+1].delta_dot; // delta dot
            }else{
                x_prev.segment(i * n_states, n_states) = x_prev.segment((i - 1) * n_states, n_states);
            }

            // u_prev vector
            if (i < prev_controls.size()-1){
                u_prev(i * n_controls) = prev_controls[i+1].steering;   // steering
                u_prev(i * n_controls +1) = prev_controls[i+1].mz;      // mz
            }else{
                u_prev.segment(i * n_controls, n_controls) = u_prev.segment((i - 1) * n_controls, n_controls);
            }

            // Discrete model matrices Ad Bd Cd
            auto prev_state = x_prev.segment(i * n_states, n_states);
            auto prev_u = u_prev.segment(i * n_controls, n_controls);

            model->getDiscreteMatrices(prev_state, prev_u, vx[i], Ad[i], Bd[i], Cd[i]);

            if(cfg.verbose){
                std::cout << "At step " << i << " : vx: " << vx[i] << " phi: " << x_prev[i*6+2] << " delta: " << x_prev[i*6+4] << std::endl;
                std::cout << "Ad[" << i << "]:\n" << Ad[i] << std::endl;
                std::cout << "Bd[" << i << "]:\n" << Bd[i] << std::endl;
                std::cout << "Cd[" << i << "]:\n" << Cd[i].transpose() << std::endl;
            }

            // Sanity check
            if(!Ad[i].allFinite() || !Bd[i].allFinite() || !Cd[i].allFinite()){
                std::cout << "NaN in model matrices at step " << i << std::endl;
            }
        }
        }   

        // Fill matrix T
        {PROFC_NODE("createModelMatrices_T")
        temp6x6 = Ad[0];
        T.block(0, 0, n_states, n_states) = temp6x6;
        for (int i = 1; i < n_horizon; ++i)
        {
            temp6x6.noalias() = Ad[i] * temp6x6;
            T.block(i * n_states, 0, n_states, n_states) = temp6x6;
        }
        }

        // Fill matrix S
        {PROFC_NODE("createModelMatrices_S")
        for (int j = 0; j < n_horizon; ++j)
        {
            Eigen::Matrix<double, 6, 2> AB = Bd[j];

            for (int i = j; i < n_horizon; ++i)
            {
                if (i > j)
                {
                    AB.noalias() = Ad[i] * AB;
                }

                S.block(i * n_states, j * n_controls, n_states, n_controls) = AB;
            }
        }
        }

        // Fill matrix W
        {PROFC_NODE("createModelMatrices_W")
        W.setZero();
        for (int j = 0; j < n_horizon; ++j)
        {
            Eigen::VectorXd Aprop_C = Cd[j];

            for (int i = j; i < n_horizon; ++i)
            {
                if (i > j)
                    Aprop_C.noalias() = Ad[i] * Aprop_C;

                W.segment(i*n_states, n_states) += Aprop_C;
            }
        }
        }

        if(cfg.verbose){
            std::cout << "Size of x0: " << x0.size() << std::endl;
            std::cout << "Size of x_ref: " << x_ref.size() << std::endl;
            std::cout << "Size of x_prev: " << x_prev.size() << std::endl;
            std::cout << "Size of u_prev: " << u_prev.size() << std::endl;
            std::cout << "Size of Ad: " << Ad.size() << ", each of size: " << Ad[0].rows() << "x" << Ad[0].cols() << std::endl;
            std::cout << "Size of Bd: " << Bd.size() << ", each of size: " << Bd[0].rows() << "x" << Bd[0].cols() << std::endl;
            std::cout << "Size of Cd: " << Cd.size() << ", each of size: " << Cd[0].size() << std::endl;
            std::cout << "Size of T: " << T.rows() << "x" << T.cols() << std::endl;
            std::cout << "Size of S: " << S.rows() << "x" << S.cols() << std::endl;
            std::cout << "Size of W: " << W.size() << std::endl;
        }
    }

    // Solve the optimisation problem
    Controls solve(Control u_prev_iter)
    {
        PROFC_NODE_

        Eigen::DiagonalMatrix<double, Eigen::Dynamic> Q(q_diag);

        d.setZero();
        d(0) = -u_prev_iter.steering;
        d(1) = -u_prev_iter.mz;
        
        H = 2.0 * (S.transpose() * Q * S + R_);

        g.noalias() = (2 * (x0.transpose() * T.transpose() - x_ref.transpose()) * Q * S).transpose();

        // Solution without constraints
            // u_opt = H.ldlt().solve(-g); // Cholesk variant (for positive and negative defined matrices)
            // u_opt = H.llt().solve(-g); // Cholesky decomposition (need to find if H is positive define)

        // Solution with constraints using HPIPM solver
            solver.solve(H, g, S, T, x0);
            u_opt = solver.getSolution();

        Controls optimal_controls(n_horizon);
        for (size_t i = 0; i < n_horizon; ++i){
            optimal_controls[i].steering = u_opt(i * n_controls);
            optimal_controls[i].mz = u_opt(i * n_controls + 1);
        }

        if(cfg.verbose){
            std::cout << "Optimal control sequence (first 5 points): " << std::endl;
            for (size_t i = 0; i < std::min(size_t(5), optimal_controls.size()); ++i){
                std::cout << "Control " << i << ": steering: " << optimal_controls[i].steering << std::endl;
            }
        }

        return optimal_controls;
    }

    // Predict the following states given initial state and controls
    States predict_states(const State &x_0, const Controls &controls, const States& x_ref){
        PROFC_NODE_

        States predicted_states(n_horizon);

        u_vec.setZero();
        for (size_t i = 0; i < cfg.mpc.n_horizon; ++i){
            u_vec(i * n_controls) = controls[i].steering;
            u_vec(i * n_controls + 1) = controls[i].mz;
        }

        x_pred.noalias() = S * u_vec + T * x0; // + W;

        for (size_t i = 0; i < n_horizon; ++i){
            predicted_states[i].y = x_pred(i * n_states);
            predicted_states[i].vy = x_pred(i * n_states + 1);
            predicted_states[i].psi = x_pred(i * n_states + 2);
            predicted_states[i].r = x_pred(i * n_states + 3);
            predicted_states[i].delta = x_pred(i * n_states + 4);
            predicted_states[i].delta_dot = x_pred(i * n_states + 5);

            predicted_states[i].vx = vx[i+1]; // vx is not predicted by the model

            if(i == 0){
                predicted_states[i].x = x_0.x;
            }else{
                predicted_states[i].x = predicted_states[i-1].x
                                        + (predicted_states[i].vx * cos(predicted_states[i].psi)
                                        + predicted_states[i].vy * sin(predicted_states[i].psi)) * cfg.mpc.Ts;
            }
        }

        return predicted_states;
    }

    /////////////////////////////////////////////////////////////////////////
    //-------------------------- Auxiliar functions  ----------------------//

    // Safety function for steering command
    double steeringSafety(const double &steering, const State &car_state)
    {
        if (std::isnan(steering))
            return 0.0;
        else if (car_state.vx < 0.3)
            return 0.0;
        else if (steering > cfg.mpc.max_steering)
            return cfg.mpc.max_steering;
        else if (steering < -cfg.mpc.max_steering)
            return -cfg.mpc.max_steering;
        else
            return steering;
    }

    // Compute model error
    double compute_model_error(const States pred, const States meas)
    {
        double err_y = 0.0;
        double err_vy = 0.0; 
        double err_psi = 0.0;
        double err_r = 0.0;
        double err_delta = 0.0;

        size_t n_pred = pred.size();
        size_t n_meas = meas.size();
        size_t n = std::min(n_pred, n_meas);

        if(n_pred != n_meas){
            std::cerr << "Error computing model error: sizes don't match: pred(" << pred.size() << " meas(" << meas.size() << ")" <<  std::endl;
        }

        for(size_t i = 0; i < n; i++){

            double dy     = pred[i].y     - meas[i].y;
            double dvy    = pred[i].vy    - meas[i].vy;
            double dpsi   = wrap_angle(pred[i].psi - meas[i].psi);
            double dr     = pred[i].r     - meas[i].r;
            double ddelta = pred[i].delta - meas[i].delta;

            err_y     += (dy * dy)         / (cfg.mpc.scale_y   * cfg.mpc.scale_y);
            err_vy    += (dvy * dvy)       / (cfg.mpc.scale_vy  * cfg.mpc.scale_vy);
            err_psi   += (dpsi * dpsi)     / (cfg.mpc.scale_psi * cfg.mpc.scale_psi);
            err_r     += (dr * dr)         / (cfg.mpc.scale_r   * cfg.mpc.scale_r);
            err_delta += (ddelta * ddelta) / (cfg.mpc.scale_st  * cfg.mpc.scale_st);
        }

        double error = err_y + err_vy + err_psi + err_r + err_delta;

        if(cfg.verbose){
            std::cout << "Model error across " << n << " steps: " << error << std::endl;
            std::cout << "\t err_y: " << err_y << std::endl;
            std::cout << "\t err_vy: " << err_vy << std::endl;
            std::cout << "\t err_psi: " << err_psi << std::endl;
            std::cout << "\t err_r: " << err_r << std::endl;
            std::cout << "\t err_delta: " << err_delta << std::endl;
        }

        return error;
    }

    // Wrap angle for model evaluation
    double wrap_angle(double a)
    {
        a = std::fmod(a + M_PI, 2.0 * M_PI);

        if(a < 0)
            a += 2.0 * M_PI;

        return a - M_PI;
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

    // Read csv and load as eigen to debug
    Eigen::VectorXd loadCsvRowAsEigen(const std::string& path, int row_index = -1)
    {
        std::ifstream file(path);

        if (!file.is_open()) {
            throw std::runtime_error("Could not open CSV file: " + path);
        }

        std::string line;
        std::vector<std::string> lines;

        while (std::getline(file, line)) {
            if (!line.empty()) {
                lines.push_back(line);
            }
        }

        if (lines.empty()) {
            throw std::runtime_error("CSV file is empty: " + path);
        }

        std::string selected_line;

        if (row_index < 0) {
            selected_line = lines.back();  // last row
        } else {
            if (row_index >= static_cast<int>(lines.size())) {
                throw std::runtime_error("Requested row does not exist in CSV: " + path);
            }

            selected_line = lines[row_index];
        }

        std::stringstream ss(selected_line);
        std::string cell;
        std::vector<double> values;

        while (std::getline(ss, cell, ',')) {
            if (!cell.empty()) {
                values.push_back(std::stod(cell));
            }
        }

        Eigen::VectorXd v(values.size());

        for (int i = 0; i < static_cast<int>(values.size()); ++i) {
            v(i) = values[i];
        }

        return v;
    }
  
  public:
    // Constructor
    MPC(): cfg(Config::getInstance()) {}

    void initialize()
    {    
        std::cout << "Initializing MPC..." << std::endl;

        // Model
        model = std::make_unique<LtvModel>(); // LtvModel AnfisModel
        model->initialize();

        // Save recurrent parameters
        n_horizon = cfg.mpc.n_horizon;

        // Initialize weights matrices
        q_diag.resize(n_states * n_horizon);
        R.resize(n_controls, n_controls);
        Rd.resize(n_controls, n_controls);
        R_.resize(n_horizon * n_controls, n_horizon * n_controls);
        Rd_.resize(n_horizon * n_controls, n_horizon * n_controls);
        D.resize(n_horizon * n_controls, n_horizon * n_controls);

        // Inputs
        x0.resize(n_states);
        x_ref.resize(n_states * n_horizon);
        x_prev.resize(n_states * n_horizon);
        u_prev.resize(n_controls * n_horizon);
        vx.resize(n_horizon);

        // Discrete models
        Ad.resize(n_horizon);
        Bd.resize(n_horizon);
        Cd.resize(n_horizon);
        for (size_t i = 0; i < n_horizon; ++i){
            Ad[i].resize(n_states, n_states);
            Bd[i].resize(n_states, n_controls);
            Cd[i].resize(n_states);
        }

        // Intermediate calculations
        Aprod.resize(n_states, n_states);
        temp6x6.resize(n_states, n_states);
        wk.resize(n_states);

        // Condensed model
        S.resize(n_horizon * n_states, n_horizon * n_controls);
        T.resize(n_horizon * n_states, n_states);
        W.resize(n_horizon * n_states); 
        S.setZero();
        T.setZero();
        W.setZero();

        // Solver matrices
        H.resize(n_horizon * n_controls, n_horizon * n_controls);
        g.resize(n_horizon * n_controls);
        u_opt.resize(n_horizon * n_controls);
        d.resize(n_horizon * n_controls);

        // Prediction
        x_pred.resize(n_horizon * n_states);
        u_vec.resize(n_horizon * n_controls);

        createWeights();

        firstIteration = true;

        solver.setParams(cfg.mpc.max_steering, cfg.mpc.max_steering_dot, cfg.mpc.max_mz, n_states, n_horizon, n_controls, cfg.verbose);
        std::cout << "MPC initialized" << std::endl;
    }

    ~MPC() = default;

    void compute_mpc(const State &car_state, const Control last_control, const States &local_ref, const States &prev_states, const Controls &prev_controls, States &predicted_states, Controls &optimal_controls)
    {
        PROFC_NODE_

        if (cfg.verbose){
            std::cout << "--- Compute MPC ---" << std::endl;
            std::cout << "Current state: y: " << car_state.y << ", vy: " << car_state.vy << ", psi: " << car_state.psi << ", r: " << car_state.r << ", delta: " << car_state.delta << ", delta_dot: " << car_state.delta_dot << std::endl;
            std::cout << "Size of local reference: " << local_ref.size() << std::endl;
            std::cout << "Size of previous states: " << prev_states.size() << std::endl;
            std::cout << "Size of previous controls: " << prev_controls.size() << std::endl;
        }

        createReference(local_ref);
        createModelMatrices(car_state, prev_states, prev_controls);
        optimal_controls = solve(last_control);

        // Ensure safety
        for (size_t i = 0; i < optimal_controls.size(); ++i){
            optimal_controls[i].steering = steeringSafety(optimal_controls[i].steering, car_state);
        }

        predicted_states = predict_states(car_state, optimal_controls, local_ref);
        firstIteration = false;

        if(cfg.save_debug){
            appendSingleRowToCSV(x0, "x0");
            appendSingleRowToCSV(x_ref, "x_ref");
            appendSingleRowToCSV(x_prev, "x_prev");
            appendSingleRowToCSV(vx, "vx");
            appendSingleRowToCSV(u_prev, "u_prev");
            appendSingleRowToCSV(T, "T");
            appendSingleRowToCSV(S, "S");
            appendSingleRowToCSV(W, "W");
            appendSingleRowToCSV(d, "d");
            appendSingleRowToCSV(H, "H");
            appendSingleRowToCSV(g, "g");
            appendSingleRowToCSV(u_opt, "u_opt");
            appendSingleRowToCSV(x_pred, "x_pred");
        }
    }

    float compute_prediction(const State &first_state, const Controls &controls, const States &states)
    {
        PROFC_NODE_

        if (cfg.verbose){
            std::cout << "--- Compute Prediction ---" << std::endl;
            std::cout << "Current state: y: " << first_state.y << ", vy: " << first_state.vy << ", psi: " << first_state.psi << ", r: " << first_state.r << ", delta: " << first_state.delta << ", delta_dot: " << first_state.delta_dot << std::endl;
            std::cout << "Size of previous states: " << states.size() << std::endl;
            std::cout << "Size of previous controls: " << controls.size() << std::endl;
        }

        createReference(states);
        createModelMatrices(first_state, states, controls);
        States prediction = predict_states(first_state, controls, states);

        double model_error = compute_model_error(prediction, states);

        if(cfg.save_debug){
            // appendSingleRowToCSV(x0, "x0");
            // appendSingleRowToCSV(x_prev, "x_prev");
            // appendSingleRowToCSV(vx, "vx");
            // appendSingleRowToCSV(u_prev, "u_prev");
            // appendSingleRowToCSV(T, "T");
            // appendSingleRowToCSV(S, "S");
            // appendSingleRowToCSV(W, "W");
            // appendSingleRowToCSV(u_vec, "u_opt");
            // appendSingleRowToCSV(x_pred, "x_pred");
        }

        return model_error;
    }

    void update_config()
    {
        createWeights();
    }
};