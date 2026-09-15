#include "mpc.hpp"

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace
{
void configure(const std::string& model_name)
{
    Config& cfg = Config::getInstance();
    cfg.verbose = false;
    cfg.save_debug = false;
    cfg.profile = false;
    cfg.share_path = std::string(FUZZY_MPC_SOURCE_DIR) + "/";

    cfg.mpc.model = model_name;
    cfg.mpc.Ts = 0.02;
    cfg.mpc.latency = 0.0;
    cfg.mpc.n_horizon = 60;
    cfg.mpc.n_evaluation = 60;
    cfg.mpc.max_steering = 0.38;
    cfg.mpc.max_delta = 0.45;
    cfg.mpc.max_steering_dot = 1.396;
    cfg.mpc.scale_y = 0.1;
    cfg.mpc.scale_vy = 0.1;
    cfg.mpc.scale_psi = 0.05;
    cfg.mpc.scale_r = 0.05;
    cfg.mpc.scale_st = 0.2;
    cfg.mpc.scale_dst = 0.001;
    cfg.mpc.q_lat = 200.0;
    cfg.mpc.q_vy = 0.0;
    cfg.mpc.q_phi = 0.0;
    cfg.mpc.q_r = 1.0;
    cfg.mpc.q_delta = 0.0;
    cfg.mpc.q_delta_dot = 0.0;
    cfg.mpc.p_lat = 1000.0;
    cfg.mpc.p_vy = 0.0;
    cfg.mpc.p_phi = 0.0;
    cfg.mpc.p_r = 0.0;
    cfg.mpc.p_delta = 0.0;
    cfg.mpc.p_delta_dot = 0.0;
    cfg.mpc.r_st = 1.0;
    cfg.mpc.rd_st = 2.0;

    cfg.car.m = 207.0;
    cfg.car.I = 129.024;
    cfg.car.Lf = 0.765;
    cfg.car.Lr = 0.765;
    cfg.car.Bf = 10.5507;
    cfg.car.Br = 10.5507;
    cfg.car.Cf = -1.2705;
    cfg.car.Cr = -1.2705;
    cfg.car.Df = 1104.0;
    cfg.car.Dr = 1281.5;
    cfg.car.steering_damp = 0.5;
    cfg.car.steering_omega = 16.0;
}
}

int main(int argc, char** argv)
{
    if (argc != 2) {
        std::cerr << "Usage: mpc_smoke_test <ltv|anfis_direct>\n";
        return 2;
    }
    configure(argv[1]);

    Config& cfg = Config::getInstance();
    MPC controller;
    controller.initialize();

    State state{};
    state.y = 0.20;
    state.psi = 0.04;
    state.vx = 8.0;
    state.vy = 0.10;
    state.r = 0.02;

    States reference(cfg.mpc.n_horizon);
    for (int i = 0; i < cfg.mpc.n_horizon; ++i) {
        reference[i].x = (i + 1) * state.vx * cfg.mpc.Ts;
        reference[i].vx = state.vx;
    }
    States previous_states = reference;
    Controls previous_controls(cfg.mpc.n_horizon, Control{0.0});
    States prediction;
    Controls controls;
    controller.compute_mpc(state, Control{0.0}, reference,
        previous_states, previous_controls, prediction, controls);

    if (prediction.size() != static_cast<size_t>(cfg.mpc.n_horizon) ||
        controls.size() != static_cast<size_t>(cfg.mpc.n_horizon)) {
        std::cerr << "Unexpected prediction dimensions.\n";
        return 1;
    }
    const double max_first_step = cfg.mpc.max_steering_dot * cfg.mpc.Ts + 1e-8;
    if (!std::isfinite(controls.front().steering) ||
        std::abs(controls.front().steering) > max_first_step) {
        std::cerr << "Invalid first steering command: "
                  << controls.front().steering << "\n";
        return 1;
    }
    for (size_t i = 0; i < controls.size(); ++i) {
        if (!std::isfinite(controls[i].steering) ||
            std::abs(controls[i].steering) > cfg.mpc.max_steering + 1e-8) {
            std::cerr << "Invalid steering command at " << i << "\n";
            return 1;
        }
        if (i > 0 && std::abs(controls[i].steering - controls[i - 1].steering)
                > max_first_step) {
            std::cerr << "Steering-rate constraint violated at " << i << "\n";
            return 1;
        }
        const State& x = prediction[i];
        if (!std::isfinite(x.x) || !std::isfinite(x.y) ||
            !std::isfinite(x.psi) || !std::isfinite(x.vy) ||
            !std::isfinite(x.r) || !std::isfinite(x.delta) ||
            !std::isfinite(x.delta_dot)) {
            std::cerr << "Non-finite prediction at " << i << "\n";
            return 1;
        }
        if (std::abs(x.delta) > cfg.mpc.max_delta + 1e-7 ||
            std::abs(x.delta_dot) > cfg.mpc.max_steering_dot + 1e-7) {
            std::cerr << "Predicted actuator-state constraint violated at "
                      << i << "\n";
            return 1;
        }
    }

    std::cout << argv[1] << " MPC smoke test passed; first steering = "
              << controls.front().steering << " rad\n";
    return 0;
}
