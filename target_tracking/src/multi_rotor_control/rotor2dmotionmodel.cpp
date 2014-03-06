#include "rotor2dmotionmodel.h"
#include <Eigen/Cholesky>
#include <Eigen/Geometry>

namespace ranav {

Rotor2dMotionModel::Rotor2dMotionModel() :
  dt(0)
{
}

Rotor2dMotionModel::~Rotor2dMotionModel() {
}

void Rotor2dMotionModel::init(const TParam &p) {
  nA = p("multi_rotor_control/numAgents").toInt();
  nT = p("multi_rotor_control/numTargets").toInt();
  dt = p("estimation/motionDt").toDouble();
  agentStateDim = 3;
  agentControlDim = 3;
  targetStateDim = 4;
  stateDim = agentStateDim*nA + targetStateDim*nT;
  controlDim = agentControlDim*nA;
  noiseDim = 3*nA + 2*nT;
  A = Eigen::MatrixXd::Identity(stateDim, stateDim);
  for (unsigned int i=0; i<nT; ++i) {
    A.block(agentStateDim*nA+targetStateDim*i, agentStateDim*nA+targetStateDim*i+2, 2, 2) = Eigen::MatrixXd::Identity(2, 2)*dt; // constant velocity model for target
    A.block(agentStateDim*nA+targetStateDim*i+2, agentStateDim*nA+targetStateDim*i+2, 2, 2) = Eigen::MatrixXd::Identity(2, 2)*0.7; // exponential decay of velocity
  }
  B = Eigen::MatrixXd::Identity(stateDim, controlDim)*dt;
  V = Eigen::MatrixXd::Zero(stateDim, noiseDim);
  V.block(0, 0, agentStateDim*nA, agentStateDim*nA) = Eigen::MatrixXd::Identity(agentStateDim*nA, agentStateDim*nA);
  for (unsigned int i=0; i<nT; ++i) {
    V.block(agentStateDim*nA+targetStateDim*i+2, 3*nA+4*i, 2, 2) = Eigen::MatrixXd::Identity(2, 2);
  }
  double motionStdDev = p("multi_rotor_control/motionStdDev").toDouble();
  Eigen::VectorXd noiseVec = Eigen::VectorXd::Constant(noiseDim, motionStdDev*motionStdDev * dt);
  noiseVec.segment(noiseVec.size()-2*nT, 2*nT).setConstant(0.4 * dt);
  noiseCov = noiseVec.asDiagonal();
  noiseCovSqrt = noiseCov.llt().matrixL();
}

Eigen::VectorXd Rotor2dMotionModel::move(const Eigen::VectorXd &state, const Eigen::VectorXd &control, const Eigen::VectorXd &noise) const {
  return A*state + jacobianControl(state, control)*control + V*noise;
}

Eigen::MatrixXd Rotor2dMotionModel::jacobianState(const Eigen::VectorXd &state, const Eigen::VectorXd &control, const Eigen::VectorXd &) const {
  return A;
}

Eigen::MatrixXd Rotor2dMotionModel::jacobianControl(const Eigen::VectorXd &state, const Eigen::VectorXd &, const Eigen::VectorXd &) const {
  Eigen::MatrixXd res(B);
  for (unsigned int i=0; i<nA; ++i) {
    res.block(i*agentStateDim, i*agentControlDim, 2, 2) = Eigen::Rotation2Dd(state(i*agentStateDim+2)).toRotationMatrix()*dt;
  }
  return res;
}

Eigen::MatrixXd Rotor2dMotionModel::jacobianNoise(const Eigen::VectorXd &, const Eigen::VectorXd &, const Eigen::VectorXd &) const {
  return V;
}

} /* namespace ranav */
