#include "motionmodel.h"
#include "random.h"

namespace ranav {

Eigen::VectorXd MotionModel::move(const Eigen::VectorXd &state, const Eigen::VectorXd &control) const {
  return move(state, control, Eigen::VectorXd::Zero(noiseDim));
}

Eigen::VectorXd MotionModel::sampleNoise(const Eigen::VectorXd &, const Eigen::VectorXd &) const {
  return Random::multivariateGaussian(noiseCov, &noiseCovSqrt);
}

Eigen::MatrixXd MotionModel::getNoiseCov(const Eigen::VectorXd &, const Eigen::VectorXd &) const {
  return noiseCov;
}

Eigen::MatrixXd MotionModel::jacobianState(const Eigen::VectorXd &state, const Eigen::VectorXd &control) const {
  return jacobianState(state, control, Eigen::VectorXd::Zero(noiseDim));
}

Eigen::MatrixXd MotionModel::jacobianControl(const Eigen::VectorXd &state, const Eigen::VectorXd &control) const {
  return jacobianControl(state, control, Eigen::VectorXd::Zero(noiseDim));
}

Eigen::MatrixXd MotionModel::jacobianNoise(const Eigen::VectorXd &state, const Eigen::VectorXd &control) const {
  return jacobianNoise(state, control, Eigen::VectorXd::Zero(noiseDim));
}

} /* namespace ranav */
