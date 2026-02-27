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
#include <iostream>

using namespace Eigen;
using namespace std;

class Solver{
  public:
    Solver() = default;
    ~Solver() = default;

    void solve(const MatrixXd& H, const VectorXd& g, const MatrixXd& S, const MatrixXd& T, const VectorXd& x0){
        
        H_ = H;
        g_ = g;
        S_ = S;
        T_ = T;
        x0_ = x0;

        double *H_pointer = new double[n_horizon_ * n_horizon_];
        double *g_pointer = new double[n_horizon_];
        int *idxb = new int[n_horizon_];
        double *lb_pointer = new double[n_horizon_];
        double *ub_pointer = new double[n_horizon_];
        double *C_pointer = new double[n_horizon_ * n_states_ * n_horizon_];
        double *lg_pointer = new double[n_horizon_ * n_states_];
        double *ug_pointer = new double[n_horizon_ * n_states_];

        fillBounds();
        for (int i = 0; i < n_horizon_; ++i)
            idxb[i] = i;

        translateMatrixToArray(H_, H_pointer);
        translateVectorToArray(g_, g_pointer);
        translateMatrixToArray(S_, C_pointer);
        translateVectorToArray(lg_, lg_pointer);
        translateVectorToArray(ug_, ug_pointer);
        translateVectorToArray(lb_, lb_pointer);
        translateVectorToArray(ub_, ub_pointer);

        int hpipm_status;
        int rep, nrep = 10;
        hpipm_timer timer;

        hpipm_size_t dim_size = d_dense_qp_dim_memsize();
        void *dim_mem = malloc(dim_size);
        struct d_dense_qp_dim dim;
        d_dense_qp_dim_create(&dim, dim_mem);
        // d_dense_qp_dim_set_all(nv_, ne_, nb_, ng_, nsb_, nsg_, &dim); No compila amb aixo 
        d_dense_qp_dim_set_all(nv_, ne_, nb_, ng_, nsb_, &dim);


        hpipm_size_t qp_size = d_dense_qp_memsize(&dim);
        void *qp_mem = malloc(qp_size);
        struct d_dense_qp qp;
        d_dense_qp_create(&dim, &qp, qp_mem);
        d_dense_qp_set_H(H_pointer, &qp);
        d_dense_qp_set_g(g_pointer, &qp);
        d_dense_qp_set_idxb(idxb, &qp);
        d_dense_qp_set_ub(ub_pointer, &qp);
        d_dense_qp_set_lb(lb_pointer, &qp);
        d_dense_qp_set_C(C_pointer, &qp);
        d_dense_qp_set_ug(ug_pointer, &qp);
        d_dense_qp_set_lg(lg_pointer, &qp);

        hpipm_size_t qp_sol_size = d_dense_qp_sol_memsize(&dim);
        void *qp_sol_mem = malloc(qp_sol_size);
        struct d_dense_qp_sol qp_sol;
        d_dense_qp_sol_create(&dim, &qp_sol, qp_sol_mem);

        hpipm_size_t ipm_arg_size = d_dense_qp_ipm_arg_memsize(&dim);
        void *ipm_arg_mem = malloc(ipm_arg_size);
        struct d_dense_qp_ipm_arg arg;
        d_dense_qp_ipm_arg_create(&dim, &arg, ipm_arg_mem);

        enum hpipm_mode mode = SPEED; // BALANCE;
        d_dense_qp_ipm_arg_set_default(mode, &arg);

        double d_tol_stat = 1e-3;
        double d_tol_eq = 1e-3;
        double d_tol_ineq = 1e-3;
        double d_tol_comp = 1e-3;
        int d_iter_max = 50;
        double d_mu0 = 1e2;
        int d_comp_res_exit = 1;
        int kkt_fact_alg = 0;
        int remove_lin_dep_eq = 0;
        double lam_min = 1e-16;
        double t_min = 1e-16;
        double tau_min = 1e-16;
        int compute_obj = 0;
        //	int t_lam_min = 2;

        d_dense_qp_ipm_arg_set_tol_stat(&d_tol_stat, &arg);
        d_dense_qp_ipm_arg_set_tol_eq(&d_tol_eq, &arg);
        d_dense_qp_ipm_arg_set_tol_ineq(&d_tol_ineq, &arg);
        d_dense_qp_ipm_arg_set_tol_comp(&d_tol_comp, &arg);
        d_dense_qp_ipm_arg_set_iter_max(&d_iter_max, &arg);
        d_dense_qp_ipm_arg_set_mu0(&d_mu0, &arg);
        d_dense_qp_ipm_arg_set_comp_res_exit(&d_comp_res_exit, &arg);
        d_dense_qp_ipm_arg_set_kkt_fact_alg(&kkt_fact_alg, &arg);
        d_dense_qp_ipm_arg_set_remove_lin_dep_eq(&remove_lin_dep_eq, &arg);
        d_dense_qp_ipm_arg_set_lam_min(&lam_min, &arg);
        d_dense_qp_ipm_arg_set_t_min(&t_min, &arg);
        d_dense_qp_ipm_arg_set_tau_min(&tau_min, &arg);
        d_dense_qp_ipm_arg_set_compute_obj(&compute_obj, &arg);
        //	d_dense_qp_ipm_arg_set_t_lam_min(&t_lam_min, &arg);

        hpipm_size_t ipm_size = d_dense_qp_ipm_ws_memsize(&dim, &arg);
        void *ipm_mem = malloc(ipm_size);
        if (ipm_mem == NULL)
        {
            cout << "Memory allocation failed for ipm_mem!" << endl;
            return; // or handle error as appropriate
        }
        struct d_dense_qp_ipm_ws workspace;
        d_dense_qp_ipm_ws_create(&dim, &arg, &workspace, ipm_mem);

        hpipm_tic(&timer);
        for (rep = 0; rep < nrep; rep++)
        {
            // call solver
            d_dense_qp_ipm_solve(&qp, &qp_sol, &arg, &workspace);
            d_dense_qp_ipm_get_status(&workspace, &hpipm_status);
        }
        double time_ipm = hpipm_toc(&timer) / nrep;

        if (verbose_)
        {
            printf("\nHPIPM returned with flag %i.\n", hpipm_status);
            if (hpipm_status == 0)
                printf("\n -> QP solved!\n");
            else if (hpipm_status == 1)
                printf("\n -> Solver failed! Maximum number of iterations reached\n");
            else if (hpipm_status == 2)
                printf("\n -> Solver failed! Minimum step lenght reached\n");
            else if (hpipm_status == 3)
                printf("\n -> Solver failed! NaN in computations\n");
            else
                printf("\n -> Solver failed! Unknown return flag\n");
            printf("\nAverage solution time over %i runs: %e [s]\n", nrep, time_ipm);
            printf("\n\n");
        }

        double *v = new double[nv_];

        d_dense_qp_sol_get_v(&qp_sol, v);

        if (verbose_){
            printf("\nv = \n");
            d_print_mat(1, nv_, v, 1);
            d_dense_qp_sol_print(&dim, &qp_sol);
        
            int iter;
            d_dense_qp_ipm_get_iter(&workspace, &iter);
            double res_stat;
            d_dense_qp_ipm_get_max_res_stat(&workspace, &res_stat);
            double res_eq;
            d_dense_qp_ipm_get_max_res_eq(&workspace, &res_eq);
            double res_ineq;
            d_dense_qp_ipm_get_max_res_ineq(&workspace, &res_ineq);
            double res_comp;
            d_dense_qp_ipm_get_max_res_comp(&workspace, &res_comp);
            double *stat;
            d_dense_qp_ipm_get_stat(&workspace, &stat);
            int stat_m;
            d_dense_qp_ipm_get_stat_m(&workspace, &stat_m);

            printf("\nipm return = %d\n", hpipm_status);
            printf("\nipm residuals max: res_g = %e, res_b = %e, res_d = %e, res_m = %e\n", res_stat, res_eq, res_ineq,
                res_comp);
            printf("\nipm iter = %d\n", iter);
            printf(
                "\nalpha_aff\tmu_aff\t\tsigma\t\talpha_prim\talpha_dual\tmu\t\tres_stat\tres_eq\t\tres_ineq\tres_"
                "comp\tdual_"
                "gap\tobj\t\tlq fact\t\titref pred\titref corr\tlin res stat\tlin res eq\tlin res ineq\tlin res comp\n");
            d_print_exp_tran_mat(stat_m, iter + 1, stat, stat_m);
            printf("\ndense ipm time = %e [s]\n\n", time_ipm);
        }

        translateArraytoVector(delta_u_opt_, v);

        free(dim_mem);
        free(qp_mem);
        free(qp_sol_mem);
        free(ipm_arg_mem);
        free(ipm_mem);
        free(v);

        delete[] H_pointer;
        delete[] g_pointer;
        delete[] C_pointer;
        delete[] lb_pointer;
        delete[] ub_pointer;
        delete[] lg_pointer;
        delete[] ug_pointer;

    }

    VectorXd getSolution(){ return delta_u_opt_; }
    
    void setParams(const double max_steering,
                   const double max_steering_dot,
                   const int n_states,
                   const int n_horizon,
                   const int n_controls,
                   const bool verbose){
        max_steering_ = max_steering;
        max_steering_dot_ = max_steering_dot;
        n_states_ = n_states;
        n_horizon_ = n_horizon;
        n_controls_ = n_controls;
        verbose_ = false; //verbose;

        lb_.resize(n_horizon_);
        ub_.resize(n_horizon_);
        lg_.resize(n_horizon_ * n_states_);
        ug_.resize(n_horizon_ * n_states_);
        delta_u_opt_.resize(n_horizon_);

        nv_ = n_horizon_;             // Number of variables
        ne_ = 0;                      // Number of equality constraints
        nb_ = n_horizon_;             // Number of box constraints
        ng_ = n_horizon_ * n_states_; // Number of general constraints
        nsb_ = 0;                     // Number of soft box constraints
        nsg_ = 0;                     // Number of soft general constraints
    }

    void fillBounds(){
        VectorXd X_max(n_horizon_ * n_states_);

        for (int i = 0; i < n_horizon_; ++i)
        {
            // Fill states constraints
            X_max[n_states_ * i] = 1000;     // numeric_limits<double>::infinity();
            X_max[n_states_ * i + 1] = 1000; // numeric_limits<double>::infinity();
            X_max[n_states_ * i + 2] = 1000; // numeric_limits<double>::infinity();
            X_max[n_states_ * i + 3] = 1000; // numeric_limits<double>::infinity();
            X_max[n_states_ * i + 4] = max_steering_;
            X_max[n_states_ * i + 5] = max_steering_dot_;

            // Fill controls constraints
            ub_[i] = max_steering_;
            lb_[i] = -max_steering_;
        }
        VectorXd c0(n_horizon_ * n_states_);
        c0 = T_ * x0_;
        ug_ = X_max - c0;
        lg_ = -X_max - c0; // x_min = - x_max

    }
    
    void translateMatrixToArray(const MatrixXd& eigen_matrix, double* array){
        for (int i = 0; i < eigen_matrix.rows(); ++i)
        {
            for (int j = 0; j < eigen_matrix.cols(); ++j)
                array[i + j * eigen_matrix.rows()] = eigen_matrix(i, j);
        }
    }
    
    void translateVectorToArray(const VectorXd& eigen_vector, double* array){
        for (int i = 0; i < eigen_vector.size(); ++i)
            array[i] = eigen_vector[i];
    }

    void translateArraytoVector(VectorXd& eigen_vector, double* array){
        for (int i = 0; i < eigen_vector.size(); ++i)
            eigen_vector[i] = array[i];
    }

  private:
    double max_steering_, max_steering_dot_;
    int n_states_, n_horizon_, n_controls_;
    int nv_, ne_, nb_, ng_, nsb_, nsg_;
    bool verbose_;

    MatrixXd H_, S_, T_;
    VectorXd g_, x0_, lg_, ug_, lb_, ub_, delta_u_opt_;
};