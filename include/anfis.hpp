#pragma once

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <vector>
#include <string>
#include <cmath>
#include <limits>
#include <iostream>
#include <stdexcept>

class Anfis
{
public:
    explicit Anfis(const std::string& yaml_path)
    {
        std::cout << "Loading ANFIS model from " << yaml_path << std::endl;
        loadModel(yaml_path);
    }

    // Returns the output of the model for a given input
    double evaluate(const Eigen::VectorXd& input) const
    {
        if (input.size() != n_in) {
            throw std::runtime_error("Anfis::evaluate: input size does not match n_inputs");
        }

        computeWeights(input);

        double y = 0.0;

        for (int i = 0; i < n_r; i++) {
            const double f = A_rule[i].dot(input) + b_rule[i];
            y += w_n[i] * f;
        }

        return y;
    }

    // Returns the linearized model for a given input: output = A * input + b
    void getLinearModel(const Eigen::VectorXd& input, Eigen::RowVectorXd& A, double& b) const
    {
        if (input.size() != n_in) {
            throw std::runtime_error("Anfis::getLinearModel: input size does not match n_inputs");
        }

        computeWeights(input);

        A = Eigen::RowVectorXd::Zero(n_in);
        b = 0.0;

        for (int i = 0; i < n_r; i++) {
            A += w_n[i] * A_rule[i].transpose();
            b += w_n[i] * b_rule[i];
        }
    }

private:
    int n_in = 0;
    int n_r  = 0;

    // mf[input_idx][rule_idx] = [sigma, c]
    std::vector<std::vector<Eigen::Vector2d>> mf;

    // For each rule: f_i = A_rule[i] * input + b_rule[i]
    std::vector<Eigen::VectorXd> A_rule;
    std::vector<double> b_rule;

    mutable std::vector<double> w;
    mutable std::vector<double> w_n;

    void loadModel(const std::string& path)
    {
        YAML::Node model = YAML::LoadFile(path);

        // Check yaml format
        if (!model["n_inputs"] || !model["n_rules"] || !model["premise"] || !model["consequent"]) {
            throw std::runtime_error("Anfis::loadModel: YAML missing required fields");
        }

        // Get sizes
        n_in = model["n_inputs"].as<int>();
        n_r  = model["n_rules"].as<int>();
        if (n_in <= 0 || n_r <= 0) {
            throw std::runtime_error("Anfis::loadModel: n_inputs and n_rules must be positive");
        }

        mf.resize(n_in);
        for (int j = 0; j < n_in; j++) {
            mf[j].resize(n_r);
        }

        // Load premise parameters
        const YAML::Node premise = model["premise"];
        for (int j = 0; j < n_in; j++) {
            const std::string input_name = "input" + std::to_string(j + 1);
            const YAML::Node inputNode = premise[input_name];

            if (!inputNode) {
                throw std::runtime_error("Anfis::loadModel: missing premise node " + input_name);
            }

            if (static_cast<int>(inputNode.size()) != n_r) {
                throw std::runtime_error("Anfis::loadModel: premise rule count mismatch in " + input_name);
            }

            for (int i = 0; i < n_r; i++) {
                if (!inputNode[i] || inputNode[i].size() != 2) {
                    throw std::runtime_error("Anfis::loadModel: each premise entry must be [sigma, c]");
                }

                mf[j][i] = Eigen::Vector2d(
                    inputNode[i][0].as<double>(),  // sigma
                    inputNode[i][1].as<double>()   // center
                );
            }
        }

        // Load consequent parameters
        A_rule.resize(n_r);
        b_rule.resize(n_r);
        const YAML::Node consequent = model["consequent"];
        if (static_cast<int>(consequent.size()) != n_r) {
            throw std::runtime_error("Anfis::loadModel: consequent rule count mismatch");
        }
        for (int i = 0; i < n_r; i++) {
            if (!consequent[i] || static_cast<int>(consequent[i].size()) != n_in + 1) {
                throw std::runtime_error("Anfis::loadModel: each consequent entry must have n_inputs + 1 values");
            }

            A_rule[i] = Eigen::VectorXd(n_in);

            for (int j = 0; j < n_in; j++) {
                A_rule[i](j) = consequent[i][j].as<double>();
            }

            b_rule[i] = consequent[i][n_in].as<double>();
        }

        w.resize(n_r, 0.0);
        w_n.resize(n_r, 0.0);
    }

    void computeWeights(const Eigen::VectorXd& input) const
    {
        std::vector<double> log_w(n_r, 0.0);
        double max_log_w = -std::numeric_limits<double>::infinity();

        // Work in the log domain so products of narrow Gaussian membership
        // functions cannot underflow to zero.
        for (int i = 0; i < n_r; ++i) {
            for (int j = 0; j < n_in; ++j) {
                const auto& params = mf[j][i];
                const double sigma = params(0);
                const double center = params(1);
                if (!std::isfinite(sigma) || sigma <= 0.0) {
                    throw std::runtime_error(
                        "Anfis::computeWeights: sigma must be positive");
                }
                const double normalized_distance =
                    (input(j) - center) / sigma;
                log_w[i] += -0.5 * normalized_distance * normalized_distance;
            }
            max_log_w = std::max(max_log_w, log_w[i]);
        }

        double sum_w = 0.0;
        for (int i = 0; i < n_r; ++i) {
            w[i] = std::exp(log_w[i] - max_log_w);
            sum_w += w[i];
        }

        if (!std::isfinite(sum_w) || sum_w <= 0.0) {
            throw std::runtime_error(
                "Anfis::computeWeights: invalid firing-strength sum");
        }

        for (int i = 0; i < n_r; i++) {
            w_n[i] = w[i] / sum_w;
        }
    }
};
