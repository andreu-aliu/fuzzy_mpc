#pragma once

#include <blasfeo_d_aux.h>
#include <blasfeo_d_aux_ext_dep.h>
#include <hpipm_d_dense_qp.h>
#include <hpipm_d_dense_qp_dim.h>
#include <hpipm_d_dense_qp_ipm.h>
#include <hpipm_d_dense_qp_sol.h>
#include <hpipm_d_dense_qp_utils.h>
#include <hpipm_timing.h>

#include <Eigen/Dense>

#include <cstdlib>
#include <iostream>
#include <new>
#include <stdexcept>
#include <vector>

class Solver
{
public:
    Solver() = default;
    ~Solver() = default;

    bool solve(const Eigen::MatrixXd& H,
               const Eigen::VectorXd& g,
               const Eigen::MatrixXd& S,
               const Eigen::MatrixXd& T,
               const Eigen::VectorXd& W,
               const Eigen::MatrixXd& D,
               const Eigen::VectorXd& x0,
               double previous_steering)
    {
        validateProblem(H, g, S, T, W, D, x0);

        H_ = H;
        g_ = g;
        S_ = S;
        T_ = T;
        W_ = W;
        D_ = D;
        x0_ = x0;
        fillBounds(previous_steering);

        std::vector<double> H_data(nv_ * nv_);
        std::vector<double> g_data(nv_);
        std::vector<int> idxb(nb_);
        std::vector<double> lb_data(nb_);
        std::vector<double> ub_data(nb_);
        std::vector<double> C_data(ng_ * nv_);
        std::vector<double> lg_data(ng_);
        std::vector<double> ug_data(ng_);
        std::vector<double> solution_data(nv_);

        for (int i = 0; i < nb_; ++i) {
            idxb[i] = i;
        }
        translateMatrixToArray(H_, H_data.data());
        translateVectorToArray(g_, g_data.data());
        translateMatrixToArray(C_, C_data.data());
        translateVectorToArray(lg_, lg_data.data());
        translateVectorToArray(ug_, ug_data.data());
        translateVectorToArray(lb_, lb_data.data());
        translateVectorToArray(ub_, ub_data.data());

        const hpipm_size_t dim_size = d_dense_qp_dim_memsize();
        void* dim_mem = std::malloc(dim_size);
        if (dim_mem == nullptr) {
            throw std::bad_alloc();
        }
        struct d_dense_qp_dim dim;
        d_dense_qp_dim_create(&dim, dim_mem);
        d_dense_qp_dim_set_all(nv_, ne_, nb_, ng_, nsb_, &dim);

        const hpipm_size_t qp_size = d_dense_qp_memsize(&dim);
        void* qp_mem = std::malloc(qp_size);
        const hpipm_size_t qp_sol_size = d_dense_qp_sol_memsize(&dim);
        void* qp_sol_mem = std::malloc(qp_sol_size);
        const hpipm_size_t arg_size = d_dense_qp_ipm_arg_memsize(&dim);
        void* arg_mem = std::malloc(arg_size);
        if (qp_mem == nullptr || qp_sol_mem == nullptr || arg_mem == nullptr) {
            std::free(dim_mem);
            std::free(qp_mem);
            std::free(qp_sol_mem);
            std::free(arg_mem);
            throw std::bad_alloc();
        }

        struct d_dense_qp qp;
        d_dense_qp_create(&dim, &qp, qp_mem);
        d_dense_qp_set_H(H_data.data(), &qp);
        d_dense_qp_set_g(g_data.data(), &qp);
        d_dense_qp_set_idxb(idxb.data(), &qp);
        d_dense_qp_set_lb(lb_data.data(), &qp);
        d_dense_qp_set_ub(ub_data.data(), &qp);
        d_dense_qp_set_C(C_data.data(), &qp);
        d_dense_qp_set_lg(lg_data.data(), &qp);
        d_dense_qp_set_ug(ug_data.data(), &qp);

        struct d_dense_qp_sol qp_sol;
        d_dense_qp_sol_create(&dim, &qp_sol, qp_sol_mem);

        struct d_dense_qp_ipm_arg arg;
        d_dense_qp_ipm_arg_create(&dim, &arg, arg_mem);
        const enum hpipm_mode mode = BALANCE;
        d_dense_qp_ipm_arg_set_default(mode, &arg);
        double tolerance = 1e-6;
        int iter_max = 50;
        int compute_residuals = 1;
        d_dense_qp_ipm_arg_set_tol_stat(&tolerance, &arg);
        d_dense_qp_ipm_arg_set_tol_eq(&tolerance, &arg);
        d_dense_qp_ipm_arg_set_tol_ineq(&tolerance, &arg);
        d_dense_qp_ipm_arg_set_tol_comp(&tolerance, &arg);
        d_dense_qp_ipm_arg_set_iter_max(&iter_max, &arg);
        d_dense_qp_ipm_arg_set_comp_res_exit(&compute_residuals, &arg);

        const hpipm_size_t workspace_size =
            d_dense_qp_ipm_ws_memsize(&dim, &arg);
        void* workspace_mem = std::malloc(workspace_size);
        if (workspace_mem == nullptr) {
            std::free(dim_mem);
            std::free(qp_mem);
            std::free(qp_sol_mem);
            std::free(arg_mem);
            throw std::bad_alloc();
        }
        struct d_dense_qp_ipm_ws workspace;
        d_dense_qp_ipm_ws_create(&dim, &arg, &workspace, workspace_mem);

        hpipm_timer timer;
        hpipm_tic(&timer);
        d_dense_qp_ipm_solve(&qp, &qp_sol, &arg, &workspace);
        const double solve_time = hpipm_toc(&timer);

        int hpipm_status = -1;
        d_dense_qp_ipm_get_status(&workspace, &hpipm_status);
        d_dense_qp_sol_get_v(&qp_sol, solution_data.data());
        translateArrayToVector(delta_u_opt_, solution_data.data());

        const bool solved = hpipm_status == 0 && delta_u_opt_.allFinite();
        if (!solved) {
            std::cerr << "HPIPM failed with status " << hpipm_status
                      << "; using the shifted previous control sequence.\n";
        }
        if (verbose_) {
            std::cout << "HPIPM status " << hpipm_status
                      << ", solve time " << solve_time << " s\n";
        }

        std::free(dim_mem);
        std::free(qp_mem);
        std::free(qp_sol_mem);
        std::free(arg_mem);
        std::free(workspace_mem);
        return solved;
    }

    const Eigen::VectorXd& getSolution() const { return delta_u_opt_; }

    void setParams(double max_steering,
                   double max_delta,
                   double max_steering_dot,
                   double sample_time,
                   int n_states,
                   int n_horizon,
                   int n_controls,
                   bool verbose)
    {
        if (n_controls != 1) {
            throw std::invalid_argument(
                "Solver supports the steering-only MPC formulation.");
        }
        max_steering_ = max_steering;
        max_delta_ = max_delta;
        max_steering_dot_ = max_steering_dot;
        sample_time_ = sample_time;
        n_states_ = n_states;
        n_horizon_ = n_horizon;
        n_controls_ = n_controls;
        verbose_ = verbose;

        nv_ = n_horizon_ * n_controls_;
        ne_ = 0;
        nb_ = nv_;
        // Two predicted actuator-state constraints and one command-increment
        // constraint per horizon step, matching the MATLAB quadprog problem.
        ng_ = 3 * n_horizon_;
        nsb_ = 0;

        lb_.resize(nb_);
        ub_.resize(nb_);
        lg_.resize(ng_);
        ug_.resize(ng_);
        C_.resize(ng_, nv_);
        delta_u_opt_ = Eigen::VectorXd::Zero(nv_);
    }

private:
    void fillBounds(double previous_steering)
    {
        const Eigen::VectorXd free_prediction = T_ * x0_ + W_;
        C_.setZero();

        for (int i = 0; i < n_horizon_; ++i) {
            lb_(i) = -max_steering_;
            ub_(i) = max_steering_;

            C_.row(i) = S_.row(n_states_ * i + 4);
            lg_(i) = -max_delta_ - free_prediction(n_states_ * i + 4);
            ug_(i) =  max_delta_ - free_prediction(n_states_ * i + 4);

            C_.row(n_horizon_ + i) = S_.row(n_states_ * i + 5);
            lg_(n_horizon_ + i) =
                -max_steering_dot_ - free_prediction(n_states_ * i + 5);
            ug_(n_horizon_ + i) =
                 max_steering_dot_ - free_prediction(n_states_ * i + 5);

            C_.row(2 * n_horizon_ + i) = D_.row(i);
        }

        const double max_command_step = max_steering_dot_ * sample_time_;
        lg_.tail(n_horizon_).setConstant(-max_command_step);
        ug_.tail(n_horizon_).setConstant(max_command_step);
        lg_(2 * n_horizon_) += previous_steering;
        ug_(2 * n_horizon_) += previous_steering;
    }

    void validateProblem(const Eigen::MatrixXd& H,
                         const Eigen::VectorXd& g,
                         const Eigen::MatrixXd& S,
                         const Eigen::MatrixXd& T,
                         const Eigen::VectorXd& W,
                         const Eigen::MatrixXd& D,
                         const Eigen::VectorXd& x0) const
    {
        if (H.rows() != nv_ || H.cols() != nv_ || g.size() != nv_ ||
            S.rows() != n_horizon_ * n_states_ || S.cols() != nv_ ||
            T.rows() != n_horizon_ * n_states_ || T.cols() != n_states_ ||
            W.size() != n_horizon_ * n_states_ ||
            D.rows() != nv_ || D.cols() != nv_ || x0.size() != n_states_) {
            throw std::invalid_argument("Solver received inconsistent QP dimensions.");
        }
        if (!H.allFinite() || !g.allFinite() || !S.allFinite() ||
            !T.allFinite() || !W.allFinite() || !D.allFinite() ||
            !x0.allFinite()) {
            throw std::invalid_argument("Solver received NaN or Inf.");
        }
    }

    static void translateMatrixToArray(
        const Eigen::MatrixXd& matrix, double* array)
    {
        for (Eigen::Index i = 0; i < matrix.rows(); ++i) {
            for (Eigen::Index j = 0; j < matrix.cols(); ++j) {
                array[i + j * matrix.rows()] = matrix(i, j);
            }
        }
    }

    static void translateVectorToArray(
        const Eigen::VectorXd& vector, double* array)
    {
        for (Eigen::Index i = 0; i < vector.size(); ++i) {
            array[i] = vector(i);
        }
    }

    static void translateArrayToVector(
        Eigen::VectorXd& vector, const double* array)
    {
        for (Eigen::Index i = 0; i < vector.size(); ++i) {
            vector(i) = array[i];
        }
    }

    double max_steering_ = 0.0;
    double max_delta_ = 0.0;
    double max_steering_dot_ = 0.0;
    double sample_time_ = 0.0;
    int n_states_ = 0;
    int n_horizon_ = 0;
    int n_controls_ = 0;
    int nv_ = 0;
    int ne_ = 0;
    int nb_ = 0;
    int ng_ = 0;
    int nsb_ = 0;
    bool verbose_ = false;

    Eigen::MatrixXd H_;
    Eigen::MatrixXd S_;
    Eigen::MatrixXd T_;
    Eigen::MatrixXd D_;
    Eigen::MatrixXd C_;
    Eigen::VectorXd W_;
    Eigen::VectorXd g_;
    Eigen::VectorXd x0_;
    Eigen::VectorXd lg_;
    Eigen::VectorXd ug_;
    Eigen::VectorXd lb_;
    Eigen::VectorXd ub_;
    Eigen::VectorXd delta_u_opt_;
};
