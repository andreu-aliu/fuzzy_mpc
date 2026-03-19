#pragma once

#include <Eigen/Dense>

class Model
{
public:

    virtual ~Model() = default;

    virtual void getDiscreteMatrices(
        const Eigen::VectorXd& x,
        const Eigen::VectorXd& u,
        const double vx,
        Eigen::MatrixXd& Ad,
        Eigen::MatrixXd& Bd,
        Eigen::VectorXd& Cd
    ) const=0;
};