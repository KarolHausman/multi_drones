#include "multi_drone_ekf/rotor3Dmotionmodel.h"
#include <Eigen/Cholesky>
#include <iostream>

namespace ranav {

Rotor3dMotionModel::Rotor3dMotionModel() : dt(0) {


    A = Eigen::MatrixXd::Identity(2, 2);
    B = Eigen::MatrixXd::Identity(2, 2);
    V = Eigen::MatrixXd::Zero(2, 2);
    noiseCov = Eigen::MatrixXd::Zero(3, 3);
    noiseCov(0,0) = 0.02;
    noiseCov(1,1) = 0.02;
    noiseCov(2,2) = 0.002;

    noiseDim = 3;
    stateDim = 3;
    controlDim = 3;
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
