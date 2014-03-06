#include "ekf.h"

#include "motionmodel.h"
#include "sensormodel.h"
#include <Eigen/LU>

namespace ranav {

EKF::EKF(MotionModel *m) :
  motionModel(m) {
}

EKF::~EKF() {
}

void EKF::init(const TParam &p) {
  mean = p("estimation/initialState").toVectorXd();
  covariance = Eigen::VectorXd(p("estimation/initialStdDev").toVectorXd().array().square()).asDiagonal();
}

void EKF::predict(const Eigen::VectorXd &control) {
  Eigen::MatrixXd A, W, Q;
  A = motionModel->jacobianState(mean, control);
  W = motionModel->jacobianNoise(mean, control);
  Q = motionModel->getNoiseCov(mean, control);
  mean = motionModel->move(mean, control);
  covariance = A * covariance * A.transpose() + W * Q * W.transpose();
}

void EKF::correct(const Eigen::VectorXd &measurement, const SensorModel &sensorModel) {
  Eigen::MatrixXd H, V, R;
  H = sensorModel.jacobianState(mean);
  V = sensorModel.jacobianNoise(mean);
  R = sensorModel.getNoiseCov(mean, measurement);
  Eigen::VectorXd dz = measurement - sensorModel.sense(mean);
  // normalize angular differences
  for (std::vector<unsigned int>::const_iterator it = sensorModel.getAngleDimensions().begin();
      it != sensorModel.getAngleDimensions().end(); ++it) {
    dz(*it) = atan2(sin(dz(*it)), cos(dz(*it)));
  }
  Eigen::MatrixXd K = covariance * H.transpose() * (H * covariance * H.transpose() + V * R * V.transpose()).lu().inverse();
  mean += K * dz;
  covariance = (Eigen::MatrixXd::Identity(mean.size(), mean.size()) - K * H) * covariance;
}

} /* namespace ranav */
