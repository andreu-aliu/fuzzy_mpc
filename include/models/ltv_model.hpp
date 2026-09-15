#pragma once

#include "utils/Config.hpp"
#include <as_lib/utils/Profiler.hpp>
#include <unsupported/Eigen/MatrixFunctions>
#include <stdexcept>
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

        if (x.size() != 6 || u.size() != 1) {
            throw std::invalid_argument("LtvModel expects 6 states and 1 input.");
        }

        const double vy        = x(1);
        const double psi       = x(2);

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

        // The configuration stores single-tyre Magic Formula factors. The
        // bicycle model uses positive axle cornering stiffnesses.
        const double Cf_ = 2.0 * std::abs(Df * Cf * Bf);
        const double Cr_ = 2.0 * std::abs(Dr * Cr * Br);

        const double Ts_ = cfg.mpc.Ts;
        const double vx_dynamic = std::max(vx, 3.0);

        Eigen::Matrix<double, 6, 6> A;
        A << 0, std::cos(psi), vx * std::cos(psi) - vy * std::sin(psi), 0, 0, 0,
            0, -(Cf_ + Cr_) / (m_ * vx_dynamic), 0,
            (Cr_ * lr_ - Cf_ * lf_) / (m_ * vx_dynamic) - vx_dynamic,
            Cf_ / m_, 0,
            0, 0, 0, 1, 0, 0,
            0, (Cr_ * lr_ - Cf_ * lf_) / (Iz_ * vx_dynamic), 0,
            -(Cf_ * lf_ * lf_ + Cr_ * lr_ * lr_) / (Iz_ * vx_dynamic),
            Cf_ * lf_ / Iz_, 0,
            0, 0, 0, 0, 0, 1,
            0, 0, 0, 0, - omega_ * omega_, - 2.0 * damp_ * omega_;

        Eigen::Matrix<double, 6, 1> B;
        B.setZero();
        B(5, 0) = omega_ * omega_;

        Eigen::Matrix<double, 6, 1> C = Eigen::Matrix<double, 6, 1>::Zero();
        C(0) = vx * std::sin(psi) + vy * std::cos(psi)
             - A(0, 1) * vy - A(0, 2) * psi;

        // Exact discretization of the local affine model, matching MATLAB.
        Eigen::Matrix<double, 8, 8> augmented =
            Eigen::Matrix<double, 8, 8>::Zero();
        augmented.block<6, 6>(0, 0) = A;
        augmented.block<6, 1>(0, 6) = B;
        augmented.block<6, 1>(0, 7) = C;
        const Eigen::Matrix<double, 8, 8> discrete =
            (augmented * Ts_).exp();
        Ad = discrete.block<6, 6>(0, 0);
        Bd = discrete.block<6, 1>(0, 6);
        Cd = discrete.block<6, 1>(0, 7);
    }

};
