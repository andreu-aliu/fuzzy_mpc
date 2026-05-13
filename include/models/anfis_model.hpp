#pragma once

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <memory>

#include <as_lib/utils/Profiler.hpp>
#include "utils/Config.hpp"
#include "anfis.hpp"
#include "model.hpp"

class AnfisModel : public Model
{
    std::unique_ptr<Anfis> vy_model;
    std::unique_ptr<Anfis> r_model;

    Eigen::Matrix<double, 5, 1> input_min;
    Eigen::Matrix<double, 5, 1> input_max;
    Eigen::Matrix<double, 5, 1> input_mean;
    Eigen::Matrix<double, 5, 1> input_std;

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

        for (size_t i = 0; i < 5; ++i)
        {
            input_mean(i) = mu[i].as<double>();
            input_std(i)  = sigma[i].as<double>();
            input_min(i)  = x_min[i].as<double>();
            input_max(i)  = x_max[i].as<double>();
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

        // Resize
        Ad = Eigen::MatrixXd::Zero(6, 6);
        Bd = Eigen::MatrixXd::Zero(6, 2);
        Cd = Eigen::VectorXd::Zero(6);

        // States
        const double y         = x(0);
        const double vy        = x(1);
        const double psi       = x(2);
        const double r         = x(3);
        const double delta     = x(4);
        const double delta_dot = x(5);

        const double steering_cmd = u(0);

        // Normalize ANFIS input
        Eigen::VectorXd anfis_input(5);
        anfis_input << vy, r, vx, delta, 0.0;

        Eigen::VectorXd anfis_input_n = (anfis_input - input_mean).cwiseQuotient(input_std);

        // =========================
        // Y kinematics
        // =========================
        Ad(0,0) = 1.0;
        Ad(0,1) = dt;
        Ad(0,2) = vx * dt;

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

        Eigen::Matrix<double, 1, 5> A_vy =
            A_vy_n.cwiseProduct(input_std.transpose());

        double b_vy =
            A_vy_n.cwiseProduct(input_mean.cwiseQuotient(input_std).transpose()).sum()
            + b_vy_n;

        Ad(1,1) = 1.0 + dt * A_vy(0);
        Ad(1,3) = dt * A_vy(1);
        Ad(1,4) = dt * A_vy(3);

        Cd(1) = dt * (b_vy + A_vy(2) * vx);

        // =========================
        // ANFIS r dynamics
        // =========================
        Eigen::RowVectorXd A_r_n;
        double b_r_n;

        r_model->getLinearModel(anfis_input_n, A_r_n, b_r_n);

        Eigen::Matrix<double, 1, 5> A_r =
            A_r_n.cwiseProduct(input_std.transpose());

        double b_r =
            A_r_n.cwiseProduct(input_mean.cwiseQuotient(input_std).transpose()).sum()
            + b_r_n;

        Ad(3,1) = dt * A_r(0);
        Ad(3,3) = 1.0 + dt * A_r(1);
        Ad(3,4) = dt * A_r(3);

        Cd(3) = dt * (b_r + A_r(2) * vx);

        // =========================
        // Steering dynamics (2nd order)
        // =========================
        const double damp_  = 0.5;
        const double omega_ = 16.0;

        // delta_dot = x6
        Ad(4,4) = 1.0;
        Ad(4,5) = dt;

        // delta_ddot dynamics
        Ad(5,4) = -omega_ * omega_ * dt;
        Ad(5,5) = 1.0 - 2.0 * damp_ * omega_ * dt;

        Bd(5,0) = omega_ * omega_ * dt;

        // =========================
        // Identity terms (important!)
        // =========================
        Ad(1,1) += 1.0 - 1.0; // already included above
        Ad(3,3) += 0.0;       // clarity

        // Ensure diagonal ones where needed
        Ad(1,1) += 0.0;
        Ad(3,3) += 0.0;
    }
};