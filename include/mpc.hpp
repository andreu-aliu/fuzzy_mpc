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
    Eigen::MatrixXd temp6x6;
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

    vector<Vector2d> planner_traj_frame;
    kdt::KDTree<Point> planner_tree;
    double reference_x, reference_y, reference_heading, sin_ref, cos_ref;
    double last_delta;
    double delta_dot_filtered = 0.0;
    double alpha_delta_dot = 1 - exp(-2 * M_PI * 5.0); // 5 Hz cutoff frequency

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

    // Select the references and convert it into the car's frame
    void findReferences(const Trajectory global_traj, const State car_state){
        PROFC_NODE_

        vector<int> selected_index(n_horizon + 1);
        vector<double> heading_track(n_horizon + 1);
        vector<double> consec_heading_track(n_horizon + 1);
        double transformed_heading = 0.0;
        double prev_heading = 0.0;

        Point p;
        p[0] = car_state.x;
        p[1] = car_state.y;

        createKDTree(global_traj);
        
        // Planner point closest to the car
        int next_state = planner_tree.nnSearch(p);
        selected_index[0] = next_state;
        
        // Precompute cos and sin of reference heading
        reference_x = global_traj[selected_index[0]].x;
        reference_y = global_traj[selected_index[0]].y;
        reference_heading = calcHeading(global_traj[selected_index[0]+1].x - global_traj[selected_index[0]].x,
                                               global_traj[selected_index[0]+1].y - global_traj[selected_index[0]].y);
        reference_heading = continuous(reference_heading, 0.0);
        cos_ref = cos(reference_heading);
        sin_ref = sin(reference_heading);

        std::cout << "Next state idx: " << next_state << std::endl;
        std::cout << "First idx x: " << reference_x << std::endl;
        std::cout << "First idx y: " << reference_y << std::endl;
        std::cout << "First idx psi: " << reference_heading << std::endl;
            
        Eigen::VectorXd x_ref_aux = Eigen::VectorXd::Zero(n_states * (n_horizon + 1));
        for (size_t i = 0; i < n_states * (n_horizon + 1); ++i){ // TODO: Really necessary?
            x_ref_aux(i) = 0.0;
        }

        for (size_t i = 0; i <= n_horizon; ++i){

            double x = global_traj[selected_index[i]].x;
            double y = global_traj[selected_index[i]].y;
            mpc_matrices.vx[i] = global_traj[selected_index[i]].vx;
            
            // Calculate the heading based on predicted position
            float disc_ = 0.025;
            int go2idx = static_cast<int>(mpc_matrices.vx[i] * cfg.mpc.Ts / disc_);
            int max_idx = static_cast<int>(global_traj.size()) - 1;
            int next_idx = std::min(selected_index[i] + go2idx, max_idx);
            double x_ = global_traj[next_idx].x;
            double y_ = global_traj[next_idx].y;
            
            // Calculate the heading of the trajectory
            double diff_x = x_ - x;
            double diff_y = y_ - y;
            double mu_ = calcHeading(diff_x, diff_y);

            mu_ = (i == 0) ? continuous(mu_, car_state.psi) : continuous(mu_, heading_track[i - 1]);

            heading_track[i] = mu_;
            
            // Predicted next position
            p[0] = x + mpc_matrices.vx[i] * cos(mu_) * cfg.mpc.Ts;
            p[1] = y + mpc_matrices.vx[i] * sin(mu_) * cfg.mpc.Ts;
            
            // Find the closest trajectory point to the predicted position
            next_state = planner_tree.nnSearch(p);
            next_state = (next_state <= selected_index[i]) ? selected_index[i] + 1 : next_state;

            if (i != n_horizon)
                selected_index[i + 1] = next_state; // std::min(next_state, static_cast<int>(planner.rows() - 2.0));

            // Convert planner point to car frame
            double translated_x = x - reference_x;
            double translated_y = y - reference_y;
            double rotated_x = translated_x * cos_ref + translated_y * sin_ref;
            double rotated_y = -translated_x * sin_ref + translated_y * cos_ref;
            
            planner_traj_frame[i] = Vector2d(rotated_x, rotated_y);
            
            // Calculate the heading of the trajectory, with two consecutive points
            double consec_diff_x = global_traj[selected_index[i]+1].x - global_traj[selected_index[i]].x;
            double consec_diff_y = global_traj[selected_index[i]+1].y - global_traj[selected_index[i]].y;
            double consec_mu = calcHeading(consec_diff_x, consec_diff_y);
            
            consec_mu = (i == 0) ? continuous(consec_mu, car_state.psi) : continuous(consec_mu, consec_heading_track[i - 1]);

            consec_heading_track[i] = consec_mu;
            
            // Update heading to trajectory frame
            double transformed_heading = consec_heading_track[i] - reference_heading;
            transformed_heading = (i == 0) ? continuous(transformed_heading, reference_heading) : continuous(transformed_heading, prev_heading);
            prev_heading = transformed_heading;
                                                
            // Fill the x_ref matrix
            Eigen::MatrixXd ref_matrix(n_states, 1);
            ref_matrix << planner_traj_frame[i](1),     // y
                        0,                              // vy
                        transformed_heading,        // Headin respect to reference frame
                        global_traj[selected_index[i]].w,  // Yaw rate
                        0,                              // delta
                        0;                              // delta dot

            x_ref_aux.block(i * n_states, 0, n_states, 1) = ref_matrix; // TODO: Really necessary?

        }

        // Fill x_ref vector [x_ref1 x_ref2 ... x_refN]^T, without x_ref0 which is irrelevant
        mpc_matrices.x_ref = x_ref_aux.block(n_states, 0, n_states * n_horizon, 1);

        if (cfg.save_debug){
            appendSingleRowToCSV(mpc_matrices.x_ref, "x_ref");
        }
        
        // Print the first 10 values of x_ref
        if (cfg.verbose) std::cout << "First 10 values of x_ref: \n" << mpc_matrices.x_ref.block(0, 0, 10, 1).transpose() << std::endl;
    }

    // Builds weights matrices
    void createWeights()
    {
        PROFC_NODE_

        // Q_ matrix diagonal
        Eigen::VectorXd diag_values(n_states);
        diag_values << cfg.mpc.q_lat / pow(cfg.mpc.scale_y, 2),
                       cfg.mpc.q_vy / pow(cfg.mpc.scale_vy, 2), 
                       cfg.mpc.q_phi / pow(cfg.mpc.scale_phi, 2), 
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

        // TODO: Debug only
        std::string ref_path = "/home/andreu/ros_ws/src/as/control/ltv_mpc/test/data/x_ref.csv";
        mpc_matrices.x_ref = loadCsvRowAsEigen(ref_path, 1);
        std::cout << "Size of x_ref: " << mpc_matrices.x_ref << std::endl;

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
        // Convert car state to reference frame
        double translated_x = car_state.x - reference_x;
        double translated_y = car_state.y - reference_y;
        double rotated_y = -translated_x * sin_ref + translated_y * cos_ref;
        double heading_rotated = car_state.psi - reference_heading;
        heading_rotated = continuous(heading_rotated, 0.0);

        if (firstIteration)
        last_delta = car_state.delta;
        
        delta_dot_filtered += alpha_delta_dot * ((car_state.delta - last_delta) / cfg.mpc.Ts - delta_dot_filtered);
        
        std::cout << "alpha_delta_dot: " << alpha_delta_dot << std::endl;

        // Fill initial/measured state
        m.x0(0) = rotated_y;           // y
        m.x0(1) = car_state.vy;        // vy
        m.x0(2) = heading_rotated;     // heading respect the reference frame
        m.x0(3) = car_state.r;        // yaw rate
        m.x0(4) = car_state.delta;        // Steering
        m.x0(5) = delta_dot_filtered;  // Filtered steering dot

        std::cout << m.x0 << std::endl;

        last_delta = car_state.delta;

        vector<double> prev_delta(n_horizon);
        prev_delta[0] = car_state.delta;
        if (firstIteration){
            for (size_t i = 1; i < n_horizon; ++i){
                prev_delta[i] = min(max(car_state.delta, -25.0 * M_PI / 180.0), 25.0 * M_PI / 180.0);
            }
        }else{
            for (size_t i = 1; i < n_horizon - 1; ++i){
                prev_delta[i] = min(max(prev_states[i+1].delta, -25.0 * M_PI / 180.0), 25.0 * M_PI / 180.0);
            }
            prev_delta[n_horizon-1] = min(max(prev_states[n_horizon-1].delta, -25.0 * M_PI / 180.0), 25.0 * M_PI / 180.0);
        }

        {PROFC_NODE("createModelMatrices_first")
        for (size_t i = 0; i < n_horizon; ++i){

            if (i < prev_states.size()){
                // x_prev vector
                m.x_prev.segment(i * n_states, n_states) << prev_states[i+1].y,         // y
                                                            prev_states[i+1].vy,        // vy
                                                            m.x_ref[i*6+2],       // phi
                                                            prev_states[i+1].r,         // r
                                                            prev_delta[i],     // delta
                                                            prev_states[i+1].delta_dot; // delta dot
            }else{
                m.x_prev.segment(i * n_states, n_states) = m.x_prev.segment((i - 1) * n_states, n_states);
            }

            // u_prev vector
            if (i < prev_controls.size()){
                m.u_prev(i * n_controls) = prev_controls[i+1].steering;   // steering
                m.u_prev(i * n_controls +1) = prev_controls[i+1].mz;      // mz
            }else{
                m.u_prev.segment(i * n_controls, n_controls) = m.u_prev.segment((i - 1) * n_controls, n_controls);
            }

            // Discrete model matrices Ad Bd Cd
            auto prev_state = m.x_prev.segment(i * n_states, n_states);
            auto prev_u = m.u_prev.segment(i * n_controls, n_controls);

            model->getDiscreteMatrices(prev_state, prev_u, m.vx[i], m.Ad[i], m.Bd[i], m.Cd[i]);

            if(cfg.verbose){
                std::cout << "At step " << i << " : vx: " << m.vx[i] << " phi: " << m.x_prev[i*6+2] << " delta: " << m.x_prev[i*6+4] << std::endl;
                std::cout << "Ad[" << i << "]:\n" << m.Ad[i] << std::endl;
                std::cout << "Bd[" << i << "]:\n" << m.Bd[i] << std::endl;
                std::cout << "Cd[" << i << "]:\n" << m.Cd[i].transpose() << std::endl;
            }

            // Sanity check
            if(!m.Ad[i].allFinite() || !m.Bd[i].allFinite() || !m.Cd[i].allFinite()){
                std::cout << "NaN in model matrices at step " << i << std::endl;
            }
        }
        }   

        // Fill matrix T
        {PROFC_NODE("createModelMatrices_T")
        m.temp6x6 = m.Ad[0];
        m.T.block(0, 0, n_states, n_states) = m.temp6x6;
        for (int i = 1; i < n_horizon; ++i)
        {
            m.temp6x6.noalias() = m.Ad[i] * m.temp6x6;
            m.T.block(i * n_states, 0, n_states, n_states) = m.temp6x6;
        }
        }

        // Fill matrix S
        {PROFC_NODE("createModelMatrices_S")
        for (int j = 0; j < n_horizon; ++j)
        {
            Eigen::Matrix<double, 6, 2> AB = m.Bd[j];

            for (int i = j; i < n_horizon; ++i)
            {
                if (i > j)
                {
                    AB.noalias() = m.Ad[i] * AB;
                }

                m.S.block(i * n_states, j * n_controls, n_states, n_controls) = AB;
            }
        }
        }

        // Fill matrix W
        {PROFC_NODE("createModelMatrices_W")
        m.W.setZero();
        for (int j = 0; j < n_horizon; ++j)
        {
            Eigen::VectorXd Aprop_C = m.Cd[j];

            for (int i = j; i < n_horizon; ++i)
            {
                if (i > j)
                    Aprop_C.noalias() = m.Ad[i] * Aprop_C;

                m.W.segment(i*n_states, n_states) += Aprop_C;
            }
        }
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

        Eigen::DiagonalMatrix<double, Eigen::Dynamic> Q(q_diag);

        d.setZero();
        d(0) = -u_prev_iter.steering;
        d(1) = -u_prev_iter.mz;
        
        H = 2.0 * (m.S.transpose() * Q * m.S + R_);

        g.noalias() = (2 * (m.x0.transpose() * m.T.transpose() - m.x_ref.transpose()) * Q * m.S).transpose();

        // Sanity checks
        // if(!H.allFinite()){
        //     std::cerr << "Error: H matrix contains non-finite values" << std::endl;
        // }
        // if(!g.allFinite()){
        //     std::cerr << "Error: g vector contains non-finite values" << std::endl;
        // }
        // Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(H);
        // double min_eig = eig.eigenvalues().minCoeff();
        // if (min_eig <= 0){
        //     std::cerr << "Error: H matrix is not positive definite, min eigenvalue: " << min_eig << std::endl;
        // }

        // Solution without constraints
            u_opt = H.ldlt().solve(-g); // Cholesk variant (for positive and negative defined matrices)
            // u_opt = H.llt().solve(-g); // Cholesky decomposition (need to find if H is positive define)

        // Solution with constraints using HPIPM solver
            // solver.solve(H, g, m.S, m.T, m.x0);
            // u_opt = solver.getSolution();

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

        if(cfg.save_debug){
            appendSingleRowToCSV(m.x0, "x0");
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
            u_vec(i * n_controls + 1) = controls[i].mz;
        }

        x_pred.noalias() = m.S * u_vec + m.T * m.x0; // + m.W;

        for (size_t i = 0; i < n_horizon; ++i){
            predicted_states[i].y = x_pred(i * n_states);
            predicted_states[i].vy = x_pred(i * n_states + 1);
            predicted_states[i].psi = x_pred(i * n_states + 2);
            predicted_states[i].r = x_pred(i * n_states + 3);
            predicted_states[i].delta = x_pred(i * n_states + 4);
            predicted_states[i].delta_dot = x_pred(i * n_states + 5);

            predicted_states[i].vx = m.vx[i]; // vx is not predicted by the model
            predicted_states[i].x = 0.025 * 5 * i;
        }

        std::cout << "AFTER SOLVING:" << std::endl;
        std::cout << "Y0: " << x0_vec(1) << " Delta: " << x0_vec(5) << std::endl; 
        for(size_t i = 0; i < 5; i++){
            std::cout << "Control: " << controls[i].steering << std::endl;
            std::cout << "Y: " << predicted_states[i].y << " Delta: " << predicted_states[i].delta << std::endl;
        }

        if(cfg.save_debug){
            appendSingleRowToCSV(x_pred, "x_pred");
        }

        return predicted_states;
    }

    /////////////////////////////////////////////////////////////////////////
    //-------------------------- Auxiliar functions  ----------------------//

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

    // Create kd-tree from the planner points
    void createKDTree(Trajectory traj){
        vector<Point> tree;
        for (size_t i = 0; i < traj.size(); i++)
        {
            Point p;
            p[0] = traj[i].x;
            p[1] = traj[i].y;
            tree.push_back(p);
        }
        planner_tree.build(tree);
    }

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
        m.temp6x6.resize(n_states, n_states);
        m.wk.resize(n_states);

        m.S.resize(n_horizon * n_states, n_horizon * n_controls);
        m.T.resize(n_horizon * n_states, n_states);
        m.W.resize(n_horizon * n_states);

        m.S.setZero();
        m.T.setZero();
        m.W.setZero();
    }
    
    // Safety function for steering command
    double steeringSafety(const double &steering, const State &car_state){
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

        planner_traj_frame.resize(n_horizon + 1);

        createWeights();

        firstIteration = true;

        solver.setParams(cfg.mpc.max_steering, cfg.mpc.max_steering_dot, cfg.mpc.max_mz, n_states, n_horizon, n_controls, cfg.verbose);
        std::cout << "MPC initialized" << std::endl;
    }

    ~MPC() = default;

    void compute_mpc(const State &car_state, const Control u_prev_iter, const Trajectory &global_traj, const States &x_prev, const Controls &u_prev, States &predicted_states, Controls &optimal_controls)
    {
        PROFC_NODE_

        if (cfg.verbose){
            std::cout << "Current state: y: " << car_state.y << ", vy: " << car_state.vy << ", psi: " << car_state.psi << ", r: " << car_state.r << ", delta: " << car_state.delta << ", delta_dot: " << car_state.delta_dot << std::endl;
            // std::cout << "Size of local reference: " << local_ref.size() << std::endl;
            std::cout << "Size of previous states: " << x_prev.size() << std::endl;
            std::cout << "Size of previous controls: " << u_prev.size() << std::endl;
        }

        findReferences(global_traj, car_state);
        createModelMatrices(car_state, x_prev, u_prev, mpc_matrices);
        optimal_controls = solve(mpc_matrices, u_prev_iter);

        // Ensure safety
        for (size_t i = 0; i < optimal_controls.size(); ++i){
            optimal_controls[i].steering = steeringSafety(optimal_controls[i].steering, car_state);
        }

        predicted_states = predict_states(car_state, optimal_controls, mpc_matrices);
        firstIteration = false;
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