#include "multi_drone_ekf/sensormodel.h"
#include "multi_drone_ekf/random.h"

namespace ranav {

Eigen::VectorXd SensorModel::sense(const Eigen::VectorXd &state) const {
  return sense(state, Eigen::VectorXd::Zero(noiseDim));
}

Eigen::VectorXd SensorModel::sampleNoise(const Eigen::VectorXd &, const Eigen::VectorXd &) const {
  return Random::multivariateGaussian(noiseCov, &noiseCovSqrt);
}

Eigen::MatrixXd SensorModel::getNoiseCov(const Eigen::VectorXd &, const Eigen::VectorXd &) const {
  return noiseCov;
}

Eigen::MatrixXd SensorModel::jacobianState(const Eigen::VectorXd &state) const {
  return jacobianState(state, Eigen::VectorXd::Zero(noiseDim));
}
Eigen::MatrixXd SensorModel::jacobianNoise(const Eigen::VectorXd &state) const {
  return jacobianNoise(state, Eigen::VectorXd::Zero(noiseDim));
}

} /* namespace ranav */
