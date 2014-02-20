#include "multi_drone_ekf/rotor2Dmotionmodel.h"
#include <Eigen/Cholesky>
#include <iostream>

namespace ranav {

Rotor2dMotionModel::Rotor2dMotionModel() : dt(0) {


    A = Eigen::MatrixXd::Identity(2, 2);
    B = Eigen::MatrixXd::Identity(2, 2)*dt;
    V = Eigen::MatrixXd::Zero(2, 2);
    noiseDim = 3;
    stateDim = 3;
    controlDim = 3;
}

Rotor2dMotionModel::~Rotor2dMotionModel() {
}

//void Rotor2dMotionModel::init(const TParam &p) {
//  stateDim = 6;
//  controlDim = 2;
//  noiseDim = 3;
//  dt = p("estimation/motionDt").toDouble();
//  gravity = p("rotor2d/gravity").toDouble();
//  A = Eigen::MatrixXd::Identity(stateDim, stateDim);
//  A.block(0, 3, 3, 3) = Eigen::MatrixXd::Identity(3, 3) * dt;
//  B = Eigen::MatrixXd::Zero(stateDim, controlDim);
//  B(5, 1) = dt;
//  V = Eigen::MatrixXd::Zero(stateDim, noiseDim);
//  V.block(3, 0, 3, 3) = Eigen::MatrixXd::Identity(3, 3) * dt;
//  noiseCov.resize(noiseDim, noiseDim);
//  noiseCov << 0.4, 0, 0, 0, 0.4, 0, 0, 0, 0.001; // TODO: PARAM
//  noiseCovSqrt = noiseCov.llt().matrixL();
//}

Eigen::VectorXd Rotor2dMotionModel::move(const Eigen::VectorXd &state, const Eigen::VectorXd &control, const Eigen::VectorXd &noise) const {


    Eigen::VectorXd result = state;
    result(0) = state(0) + cos(state(2)) * control(0)
            - sin(state(2)) * control(1);
    result(1) = state(1) + sin(state(2)) * control(0)
            + cos(state(2)) * control(1);
    result(2) = state(2) + control(2);

    result(2) = atan2(sin(result(2)), cos(result(2))); // normalize angle

    return result;


}

Eigen::MatrixXd Rotor2dMotionModel::jacobianState(const Eigen::VectorXd &state, const Eigen::VectorXd &control, const Eigen::VectorXd &) const {

    Eigen::MatrixXd result = Eigen::Matrix3d::Zero();
    result << 1, 0, -sin(state(2)) * control(0) - cos(state(2)) * control(1), 0, 1, cos(state(2)) * control(0) - sin(state(2)) * control(1), 0, 0, 1;

    return result;
}

Eigen::MatrixXd Rotor2dMotionModel::jacobianControl(const Eigen::VectorXd &state, const Eigen::VectorXd &, const Eigen::VectorXd &) const {
  Eigen::MatrixXd result(B);
//  result(3, 0) = -sin(state(2)) * dt;
//  result(4, 0) = cos(state(2)) * dt;
  return result;
}

Eigen::MatrixXd Rotor2dMotionModel::jacobianNoise(const Eigen::VectorXd &, const Eigen::VectorXd &, const Eigen::VectorXd &) const {
  return V;
}

} /* namespace ranav */

