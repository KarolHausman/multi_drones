#include "cam2dsensormodel.h"
#include <Eigen/Cholesky>
#include <iostream>
#include "random.h"

namespace ranav {

Cam2dSensorModel::Cam2dSensorModel(int fromId, int toId) {
  this->fromId = fromId;
  this->toId = toId;
}

Cam2dSensorModel::~Cam2dSensorModel() {
}

void Cam2dSensorModel::init(const TParam &p) {
  params = p;
  nA = params("multi_rotor_control/numAgents").toInt();
  nT = params("multi_rotor_control/numTargets").toInt();
  stateDim = 3*nA+4*nT;
  assert(fromId < (int)nA); // sensing source can't be a target
  assert(toId >= 0); // sensed object can't be the GPS
  if (fromId < 0) {
    cameraPose = Eigen::Vector3d(0, 0, 0);
    markerPose = params("multi_rotor_control/markerPose").toVectorXd();
  } else {
    cameraPose = params("multi_rotor_control/cameraPose").toVectorXd();
    markerPose = Eigen::Vector3d(0, 0, 0);
  }
  fromIdx = 3*fromId;
  toIdx = 3*toId;
  if (toId < (int)nA) {
    measurementDim = 3; // GPS/agent to agent
    angleDimensions.push_back(2);
  } else {
    measurementDim = 2; // agent to target
    toIdx = 3*nA + 4*(toId-nA);
  }
  noiseDim = measurementDim;
  H = Eigen::MatrixXd::Zero(measurementDim, stateDim);
  W = Eigen::MatrixXd::Identity(measurementDim, noiseDim);
  measurementNoise = pow(p("multi_rotor_control/measurementStdDev").toDouble(), 2);
  double measurementNoiseRot = pow(p("multi_rotor_control/measurementStdDevRot").toDouble(), 2);
  distanceNoiseFactor = pow(p("multi_rotor_control/distanceStdDevFactor").toDouble(), 2);
  visibilityRadius = p("multi_rotor_control/visibilityRadius").toDouble();
  noiseCov = Eigen::MatrixXd::Identity(noiseDim, noiseDim)*measurementNoise;
  if (noiseDim == 3) {
    noiseCov(2, 2) = measurementNoiseRot;
  }
}

bool Cam2dSensorModel::measurementAvailable(const Eigen::VectorXd &state) const {
  return sense(state).head(2).norm() < visibilityRadius;
}

Eigen::VectorXd Cam2dSensorModel::sense(const Eigen::VectorXd &state, const Eigen::VectorXd &noise) const {
  Eigen::Isometry2d tFrom = Eigen::Isometry2d::Identity();
  if (fromIdx >= 0) {
    tFrom = fromVec(state.segment(fromIdx, 3));
  }
  Eigen::Isometry2d tTo = fromVec(state.segment(toIdx, 3));
  Eigen::Isometry2d measurement = fromVec(cameraPose).inverse() * tFrom.inverse() * tTo * fromVec(markerPose);
  Eigen::Vector3d result = toVec(measurement);
  return result.head(measurementDim) + noise;
}

Eigen::VectorXd Cam2dSensorModel::sampleNoise(const Eigen::VectorXd &state, const Eigen::VectorXd &measurement) const {
  return Random::multivariateGaussian(getNoiseCov(state, measurement));
}

Eigen::MatrixXd Cam2dSensorModel::getNoiseCov(const Eigen::VectorXd &state, const Eigen::VectorXd &measurement) const {
  double dist = measurement.head(2).norm();
  return noiseCov * (1 + distanceNoiseFactor * pow(dist, 4));
}

double Cam2dSensorModel::getInformation(const Eigen::VectorXd &state, const Eigen::VectorXd &measurement) const {
  if (!measurementAvailable(state))
    return 0;
  double dist = measurement.head(2).norm();
  double noise = measurementNoise * (1 + distanceNoiseFactor * pow(dist, 4));
  return 1/sqrt(noise);
}

Eigen::MatrixXd Cam2dSensorModel::jacobianState(const Eigen::VectorXd &state, const Eigen::VectorXd &) const {
  Eigen::MatrixXd res = H;
  Eigen::Vector3d fromState(0, 0, 0);
  Eigen::Vector3d toState = state.segment(toIdx, 3);
  if (fromIdx >= 0) {
    fromState = state.segment(fromIdx, 3);
  }
  Eigen::Matrix2d R = Eigen::Rotation2Dd(fromState(2) + cameraPose(2)).toRotationMatrix().transpose();
  double a3 = fromState(2) - toState(2) + cameraPose(2);
  Eigen::Vector2d anglePart(markerPose(0)*sin(a3) - markerPose(1)*cos(a3), markerPose(0)*cos(a3) + markerPose(1)*sin(a3));
  if (fromIdx >= 0) {
    res.block(0, fromIdx, 2, 2) = -R;
    double c = cos(fromState(2)+cameraPose(2));
    double s = sin(fromState(2)+cameraPose(2));
    Eigen::Vector3d d = toState-fromState;
    res(0, fromIdx+2) = - s * d(0) + c * d(1);
    res(1, fromIdx+2) = - c * d(0) - s * d(1);
    res.block(0, fromIdx+2, 2, 1) -= anglePart;
    if (measurementDim == 3) {
      res(2, fromIdx+2) = -1;
    }
  }
  res.block(0, toIdx, 2, 2) = R;
  res.block(0, toIdx+2, 2, 1) += anglePart;
  if (measurementDim == 3) {
    res(2, toIdx+2) = 1;
  }
  return res;
}

Eigen::MatrixXd Cam2dSensorModel::jacobianNoise(const Eigen::VectorXd &, const Eigen::VectorXd &) const {
  return W;
}

Eigen::Isometry2d Cam2dSensorModel::fromVec(const Eigen::Vector3d &v) {
  return Eigen::Translation2d(v.head(2)) * Eigen::Rotation2Dd(v(2));
}

Eigen::Vector3d Cam2dSensorModel::toVec(const Eigen::Isometry2d &i) {
  return Eigen::Vector3d(i.translation()(0), i.translation()(1), Eigen::Rotation2Dd(0).fromRotationMatrix(i.linear()).angle());
}

} /* namespace ranav */
