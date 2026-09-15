#pragma once

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <memory>
#include <cmath>
#include <array>
#include <iostream>

#include <as_lib/utils/Profiler.hpp>
#include "utils/Config.hpp"
#include "anfis.hpp"
#include "model.hpp"

class AnfisModel : public Model
{
    std::unique_ptr<Anfis> vy_model;
    std::unique_ptr<Anfis> r_model;

    Eigen::Matrix<double, 4, 1> input_min;
    Eigen::Matrix<double, 4, 1> input_max;
    Eigen::Matrix<double, 4, 1> input_mean;
    Eigen::Matrix<double, 4, 1> input_std;
    double training_ts = 0.02;

public:
    AnfisModel() = default;

    void initialize() override
    {
        Config& cfg = Config::getInstance();

        // Load models
        vy_model = std::make_unique<Anfis>(cfg.share_path + "models/anfis_direct/vy.yaml");
        r_model  = std::make_unique<Anfis>(cfg.share_path + "models/anfis_direct/r.yaml");

        // Load normalization
        const std::string norm_path = cfg.share_path + "models/anfis_direct/normalization.yaml";
        YAML::Node norm = YAML::LoadFile(norm_path);

        if (!norm["normalization"])
        {
            throw std::runtime_error("Missing normalization node in " + norm_path);
        }

        const YAML::Node norm_node = norm["normalization"];

        const YAML::Node mu    = norm_node["mu"];
        const YAML::Node sigma = norm_node["sigma"];
        const YAML::Node x_min = norm_node["x_min"];
        const YAML::Node x_max = norm_node["x_max"];

        if (!mu || !sigma || !x_min || !x_max)
        {
            throw std::runtime_error("Missing normalization parameters in " + norm_path);
        }

        if (mu.size() != 4 || sigma.size() != 4 ||
            x_min.size() != 4 || x_max.size() != 4)
        {
            throw std::runtime_error(
                "ANFIS normalization must contain [vy, r, vx, delta].");
        }

        for (size_t i = 0; i < 4; ++i)
        {
            input_mean(i) = mu[i].as<double>();
            input_std(i)  = sigma[i].as<double>();
            input_min(i)  = x_min[i].as<double>();
            input_max(i)  = x_max[i].as<double>();
            if (!std::isfinite(input_mean(i)) ||
                !std::isfinite(input_std(i)) || input_std(i) <= 1e-8 ||
                !std::isfinite(input_min(i)) || !std::isfinite(input_max(i)))
            {
                throw std::runtime_error("Invalid ANFIS normalization values.");
            }
        }
        if (norm_node["training_ts"]) {
            training_ts = norm_node["training_ts"].as<double>();
        }
        if (!std::isfinite(training_ts) || training_ts <= 0.0) {
            throw std::runtime_error("Invalid ANFIS training_ts.");
        }
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
        if (!vy_model || !r_model)
        {
            throw std::runtime_error("ANFIS models not initialized");
        }

        const Config& cfg = Config::getInstance();
        const double dt = cfg.mpc.Ts;

        if (x.size() != 6 || u.size() != 1) {
            throw std::invalid_argument("AnfisModel expects 6 states and 1 input.");
        }

        Ad = Eigen::MatrixXd::Zero(6, 6);
        Bd = Eigen::MatrixXd::Zero(6, 1);
        Cd = Eigen::VectorXd::Zero(6);

        // States
        const double vy        = x(1);
        const double psi       = x(2);
        const double r         = x(3);
        const double delta     = x(4);
        const double step_scale = dt / training_ts;

        Eigen::Vector4d anfis_input;
        anfis_input << vy, r, vx, delta;
        const Eigen::Vector4d raw_input = anfis_input;

        // Detect extrapolation of trained inputs
        bool extrapolated = false;
        std::array<const char*, 4> labels = {"vy", "r", "vx", "delta"};

        for (int i = 0; i < 4; ++i)
        {
            if (anfis_input(i) < input_min(i) || anfis_input(i) > input_max(i))
            {
                extrapolated = true;
            }
        }

        if (extrapolated && cfg.verbose)
        {
            std::cerr << "\n===== ANFIS EXTRAPOLATION DETECTED =====\n";

            for (int i = 0; i < 4; ++i)
            {
                const bool out = anfis_input(i) < input_min(i) || anfis_input(i) > input_max(i);

                std::cerr << labels[i]
                        << ": value = " << anfis_input(i)
                        << " | min = " << input_min(i)
                        << " | max = " << input_max(i);

                if (out)
                    std::cerr << "  <-- OUT";

                std::cerr << "\n";
            }

            std::cerr << "========================================\n\n";
        }

        // Clamp and normalize input
        anfis_input = anfis_input.cwiseMax(input_min).cwiseMin(input_max);
        const Eigen::Vector4d active =
            ((raw_input.array() >= input_min.array()).cast<double>() *
             (raw_input.array() <= input_max.array()).cast<double>()).matrix();
        Eigen::VectorXd anfis_input_n =
            (anfis_input - input_mean).cwiseQuotient(input_std);

        if(cfg.verbose){
            std::cout << "ANFIS input (normalized): " << anfis_input_n.transpose() << std::endl;
        }

        // =========================
        // Y kinematics
        // =========================
        Ad(0, 0) = 1.0;
        Ad(0, 1) = std::cos(psi) * dt;
        Ad(0, 2) = (vx * std::cos(psi) - vy * std::sin(psi)) * dt;
        Cd(0) = dt * (
            vx * std::sin(psi)
            + vy * std::cos(psi)
            - Ad(0, 1) * vy
            - Ad(0, 2) * psi
        );

        // =========================
        // Psi kinematics
        // =========================
        Ad(2,2) = 1.0;
        Ad(2,3) = dt;

        // =========================
        // ANFIS Vy dynamics
        // =========================
        Eigen::RowVectorXd A_vy_n;
        double b_vy_n;

        vy_model->getLinearModel(anfis_input_n, A_vy_n, b_vy_n);

        const Eigen::RowVector4d A_vy =
            A_vy_n.cwiseQuotient(input_std.transpose());

        const double b_vy = b_vy_n - A_vy_n.cwiseProduct(input_mean.cwiseQuotient(input_std).transpose()).sum();
        const Eigen::Vector4d A_vy_effective =
            A_vy.transpose().cwiseProduct(active);
        const double vy_at_operating_point =
            b_vy + A_vy.dot(anfis_input);

        Ad(1, 1) = 1.0 - step_scale + step_scale * A_vy_effective(0);
        Ad(1, 3) = step_scale * A_vy_effective(1);
        Ad(1, 4) = step_scale * A_vy_effective(3);
        Cd(1) = step_scale * (vy_at_operating_point
            - A_vy_effective(0) * vy
            - A_vy_effective(1) * r
            - A_vy_effective(3) * delta);

        // =========================
        // ANFIS r dynamics
        // =========================
        Eigen::RowVectorXd A_r_n;
        double b_r_n;

        r_model->getLinearModel(anfis_input_n, A_r_n, b_r_n);

        const Eigen::RowVector4d A_r =
            A_r_n.cwiseQuotient(input_std.transpose());

        const double b_r = b_r_n - A_r_n.cwiseProduct(input_mean.cwiseQuotient(input_std).transpose()).sum();
        const Eigen::Vector4d A_r_effective =
            A_r.transpose().cwiseProduct(active);
        const double r_at_operating_point = b_r + A_r.dot(anfis_input);

        Ad(3, 1) = step_scale * A_r_effective(0);
        Ad(3, 3) = 1.0 - step_scale + step_scale * A_r_effective(1);
        Ad(3, 4) = step_scale * A_r_effective(3);
        Cd(3) = step_scale * (r_at_operating_point
            - A_r_effective(0) * vy
            - A_r_effective(1) * r
            - A_r_effective(3) * delta);

        // =========================
        // Steering dynamics (2nd order)
        // =========================
        const double damp_  = cfg.car.steering_damp;
        const double omega_ = cfg.car.steering_omega;

        // delta_dot = x6
        Ad(4,4) = 1.0;
        Ad(4,5) = dt;

        // delta_ddot dynamics
        Ad(5,4) = -omega_ * omega_ * dt;
        Ad(5,5) = 1.0 - 2.0 * damp_ * omega_ * dt;

        Bd(5,0) = omega_ * omega_ * dt;
    }
};
