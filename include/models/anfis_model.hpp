#pragma once

#include "utils/Config.hpp"
#include <as_lib/utils/Profiler.hpp>
#include "anfis.hpp"
#include "model.hpp"

class AnfisModel : public Model
{
    Anfis vy_model;
    Anfis r_model;

  public:
    AnfisModel(): 
        vy_model(Config::getInstance().mpc.debug_path + "vy_model.yaml"),
        r_model(Config::getInstance().mpc.debug_path + "r_model.yaml")
    {}

    void getDiscreteMatrices(
        const Eigen::VectorXd& x,
        const Eigen::VectorXd& u,
        Eigen::MatrixXd& Ad,
        Eigen::MatrixXd& Bd,
        Eigen::VectorXd& Cd
    ) const override
    {
        Config& cfg = Config::getInstance();
        PROFC_NODE_

    }

};