#include "multi_drone_ekf/rotor3dmotionmodel.h"
#include <Eigen/Cholesky>
#include <iostream>

namespace ranav {

Rotor3dMotionModel::Rotor3dMotionModel() {
}

Rotor3dMotionModel::~Rotor3dMotionModel()
{
}


Eigen::VectorXd Rotor3dMotionModel::downProjectControl(const Eigen::VectorXd &control)
{
    Eigen::VectorXd result(3);

    result(0) = control(0);
    result(1) = control(1);
    result(2) = control(5);

    return result;
}

};
