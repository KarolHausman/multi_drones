#include "point2dmotionmodel.h"
#include <Eigen/Cholesky>

namespace ranav {

Point2dMotionModel::Point2dMotionModel() :
  dt(0)
{
}

Point2dMotionModel::~Point2dMotionModel() {
}

void Point2dMotionModel::init(const TParam &p) {
  nA = p("multi_rotor_control/numAgents").toInt();
  nT = p("multi_rotor_control/numTargets").toInt();
  dt = p("estimation/motionDt").toDouble();
  agentStateDim = 2;
  agentControlDim = 2;
  targetStateDim = 4;
  stateDim = agentStateDim*nA + targetStateDim*nT;
  controlDim = agentControlDim*nA;
  noiseDim = 2*nA + 2*nT;
  assert(nT == 1); // not yet implemented for nT > 1
  A = Eigen::MatrixXd::Identity(stateDim, stateDim);
  A.block(agentStateDim*nA, agentStateDim*nA+2, 2, 2) = Eigen::MatrixXd::Identity(2, 2)*dt;
  B = Eigen::MatrixXd::Identity(stateDim, controlDim)*dt;
  V = Eigen::MatrixXd::Zero(stateDim, noiseDim);
  V.block(0, 0, 2*nA, 2*nA) = Eigen::MatrixXd::Identity(2*nA, 2*nA);
  V.block(agentStateDim*nA+2, 2*nA, 2, 2) = Eigen::MatrixXd::Identity(2, 2);
  double motionStdDev = p("multi_rotor_control/motionStdDev").toDouble();
  Eigen::VectorXd noiseVec = Eigen::VectorXd::Constant(noiseDim, motionStdDev*motionStdDev * dt);
  noiseVec.segment(noiseVec.size()-2, 2).setConstant(0.4 * dt);
  noiseCov = noiseVec.asDiagonal();
  noiseCovSqrt = noiseCov.llt().matrixL();
}

Eigen::VectorXd Point2dMotionModel::move(const Eigen::VectorXd &state, const Eigen::VectorXd &control, const Eigen::VectorXd &noise) const {
  return A*state + B*control + V*noise;
}

Eigen::MatrixXd Point2dMotionModel::jacobianState(const Eigen::VectorXd &state, const Eigen::VectorXd &control, const Eigen::VectorXd &) const {
  return A;
}

Eigen::MatrixXd Point2dMotionModel::jacobianControl(const Eigen::VectorXd &state, const Eigen::VectorXd &, const Eigen::VectorXd &) const {
  return B;
}

Eigen::MatrixXd Point2dMotionModel::jacobianNoise(const Eigen::VectorXd &, const Eigen::VectorXd &, const Eigen::VectorXd &) const {
  return V;
}

} /* namespace ranav */
