#pragma once

#include "utils/Config.hpp"
#include <as_lib/utils/Profiler.hpp>
#include "model.hpp"

class LtvModel : public Model
{
public:
    LtvModel() = default;

    void initialize() override
    {
        // No initialization needed for the LTV model
    }
    
    void getDiscreteMatrices(
        const Eigen::VectorXd& x,
        const Eigen::VectorXd& u,
        const double vx,
        Eigen::MatrixXd& Ad,
        Eigen::MatrixXd& Bd,
        Eigen::VectorXd& Cd
    ) const override
    {
        Config& cfg = Config::getInstance();

        // Resize matreix for safety
        Ad.resize(6, 6);
        Bd.resize(6, 2);
        Cd.resize(6);

        // States
        const double y         = x(0);
        const double vy        = x(1);
        const double psi       = x(2);
        const double r         = x(3);
        const double delta     = x(4);
        const double delta_dot = x(5);

        const double steering_cmd = u(0);

        // Car parameters
        const double m_  = cfg.car.m;
        const double Iz_ = cfg.car.I;
        const double lf_ = cfg.car.Lf;
        const double lr_ = cfg.car.Lr;

        const double Bf = cfg.car.Bf;
        const double Br = cfg.car.Br;
        const double Cf = cfg.car.Cf;
        const double Cr = cfg.car.Cr;
        const double Df = cfg.car.Df;
        const double Dr = cfg.car.Dr;

        const double damp_  = cfg.car.steering_damp;
        const double omega_ = cfg.car.steering_omega;

        const double Cf_ = Df * Cf * Bf;
        const double Cr_ = Dr * Cr * Br;

        const double Ts_ = cfg.mpc.Ts;
        const double vx_safe = std::max(vx, 1.0);

        // Continuous A matrix
        Eigen::Matrix<double,6,6> A;

        A << 0, cos(psi), vx*cos(psi), 0, 0, 0,
            0, (Cf_ * cos(delta) + Cr_) / (m_ * vx_safe), 0, ((lf_ * Cf_ * cos(delta) - lr_ * Cr_) / (m_ * vx_safe)) - vx, -Cf_ * cos(delta) / m_, 0,
            0, 0, 0, 1, 0, 0,
            0, (lf_ * Cf_ * cos(delta) - lr_ * Cr_) / (Iz_ * vx_safe), 0, (lf_ * lf_ * Cf_ * cos(delta) + lr_ * lr_ * Cr_) / (Iz_ * vx_safe), -lf_ * Cf_ * cos(delta) / Iz_, 0, 
            0, 0, 0, 0, 0, 1,
            0, 0, 0, 0, - omega_ * omega_, - 2.0 * damp_ * omega_;

        // Continuous B matrix
        Eigen::Matrix<double,6,2> B;
        B.setZero();
        B(5, 0) = omega_ * omega_;

        // Discretization (Euler)
        Eigen::Matrix<double,6,6> I = Eigen::Matrix<double,6,6>::Identity();

        Ad = I + A * Ts_;
        Bd = B * Ts_;

        // No affine term
        Cd = Eigen::VectorXd::Zero(6);
    }

};