#include "rel2dsensormodel.h"
#include <Eigen/Cholesky>
#include <iostream>
#include "random.h"

namespace ranav {

Rel2dSensorModel::Rel2dSensorModel(int fromId, int toId) {
  this->fromId = fromId;
  this->toId = toId;
}

Rel2dSensorModel::~Rel2dSensorModel() {
}

void Rel2dSensorModel::init(const TParam &p) {
  params = p;
  nA = params("multi_rotor_control/numAgents").toInt();
  nT = params("multi_rotor_control/numTargets").toInt();
  stateDim = 2*nA+4*nT;
  measurementDim = 2;
  noiseDim = 2;
  assert(nT == 1); // not yet implemented for nT > 1
  H = Eigen::MatrixXd::Zero(measurementDim, stateDim);
  if (toId >= 0)
    H.block(0, 2*toId, 2, 2) = Eigen::MatrixXd::Identity(2, 2);
  if (fromId >= 0)
    H.block(0, 2*fromId, 2, 2) = -Eigen::MatrixXd::Identity(2, 2);
  W = Eigen::MatrixXd::Identity(measurementDim, noiseDim);
  measurementNoise = pow(p("multi_rotor_control/measurementStdDev").toDouble(), 2);
  distanceNoiseFactor = pow(p("multi_rotor_control/distanceStdDevFactor").toDouble(), 2);
  visibilityRadius = p("multi_rotor_control/visibilityRadius").toDouble();
  noiseCov = Eigen::MatrixXd::Identity(noiseDim, noiseDim)*measurementNoise;
  noiseCovSqrt = noiseCov.llt().matrixL();
}

bool Rel2dSensorModel::measurementAvailable(const Eigen::VectorXd &state) const {
  return sense(state).norm() < visibilityRadius;
}

Eigen::VectorXd Rel2dSensorModel::sense(const Eigen::VectorXd &state, const Eigen::VectorXd &noise) const {
  return H*state + W*noise;
}

Eigen::VectorXd Rel2dSensorModel::sampleNoise(const Eigen::VectorXd &state, const Eigen::VectorXd &measurement) const {
  return Random::multivariateGaussian(getNoiseCov(state, measurement));
}

Eigen::MatrixXd Rel2dSensorModel::getNoiseCov(const Eigen::VectorXd &state, const Eigen::VectorXd &measurement) const {
  double dist = measurement.norm();
  return noiseCov * (1 + distanceNoiseFactor * pow(dist, 4));
}

double Rel2dSensorModel::getInformation(const Eigen::VectorXd &state, const Eigen::VectorXd &measurement) const {
  if (!measurementAvailable(state))
    return 0;
  double dist = measurement.norm();
  double noise = measurementNoise * (1 + distanceNoiseFactor * pow(dist, 4));
  return 1/sqrt(noise);
}

Eigen::MatrixXd Rel2dSensorModel::jacobianState(const Eigen::VectorXd &state, const Eigen::VectorXd &) const {
  return H;
}

Eigen::MatrixXd Rel2dSensorModel::jacobianNoise(const Eigen::VectorXd &, const Eigen::VectorXd &) const {
  return W;
}

} /* namespace ranav */
