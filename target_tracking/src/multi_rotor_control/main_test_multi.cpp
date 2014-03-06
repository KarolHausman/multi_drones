#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <unistd.h>
#include <csignal>
#include <fstream>
#include "point2dmotionmodel.h"
#include "rel2dsensormodel.h"
#include "rotor2dmotionmodel.h"
#include "cam2dsensormodel.h"
#include "ekf.h"
#include "random.h"
#include "targettrajectory.h"
#include "targettrackingcontroller.h"
#include "topology.h"
#include "tparam.h"

using namespace ranav;

bool _run = true;

void shutdown_module(int sig) {
  if(sig == SIGINT) {
    _run = false;
  }
}

int main(int argc, char **argv) {
  Random::randomize();
  std::string paramfile = "multi.ini";
  int test = 1;

  char c;
  while ((c = getopt(argc, argv, "p:h")) != EOF) {
    switch (c) {
      case 'p':
        paramfile = optarg;
        break;
      case 'h':
      default:
        std::cerr << "Usage: " << argv[0] << " [options]\n";
        std::cerr << "\nOptions:\n";
        std::cerr << "-p <file>:  use the given parameter file\n";
        std::cerr << "-h:         this help\n";
        return 1;
    }
  }

  TParam p;
  p.loadTree(paramfile);

  MultiAgentMotionModel *motionModel;
  std::string typestr = p("multi_rotor_control/type").toString();
  if (typestr == "point2d") {
    motionModel = new Point2dMotionModel();
  } else if (typestr == "rotor2d") {
    motionModel = new Rotor2dMotionModel();
  } else {
    std::cerr << "Error: Unknown type '" << typestr << "' - exiting\n";
    exit(-1);
  }
  motionModel->init(p);

  TargetTrackingController ttc;
  ttc.init(p);
  std::vector<const SensorModel*> sensorModels = ttc.getTopology().getSensorModels();

  if (test == 1) {
    // online test
    TargetTrajectory tt;
    tt.init(p);
    EKF ekf(motionModel);
    ekf.init(p);

    std::ofstream topo_out("topo.out");
    std::ofstream multi_out("multi.out");
    unsigned int nA = motionModel->getNumAgents();
    unsigned int nT = motionModel->getNumTargets();
    unsigned int aSD = motionModel->getAgentStateDim();
    unsigned int cSD = motionModel->getAgentControlDim();
    unsigned int tSD = motionModel->getTargetStateDim();

    // test output
    Eigen::VectorXd state = p("estimation/initialState").toVectorXd();
    for (unsigned int i=0; !tt.atEnd(); ++i) {
      Eigen::VectorXd mean = ekf.getMean();
      Eigen::MatrixXd cov = ekf.getCovariance();
//      Eigen::VectorXd control(nA*cSD);
//      Eigen::VectorXd targetPosition = state.segment(aSD*nA, cSD);
//      for (unsigned int i=0; i<nA; ++i) {
//        control.segment(cSD*i, cSD) = (targetPosition - state.segment(aSD*i, cSD))/p("estimation/motionDt").toDouble();
//      }
      Eigen::VectorXd control = ttc.getControlTopo(&ekf, motionModel, sensorModels);
      multi_out << aSD << " " << cSD << " " << tSD << " 0 0 " << nA << " " << nT << " 0 0 0"
          << " " << state.transpose()
          << " " << control.transpose()
          << " " << mean.transpose();
      multi_out << " " << Eigen::Map<Eigen::MatrixXd>(cov.data(), 1, cov.cols()*cov.cols());
//      for (int i=0; i<N+1; ++i) {
//        multi_out << " " << cov(2*agentStateDim, 2*agentStateDim) << " " << cov(2*agentStateDim, 2*agentStateDim+1) << " " << cov(2*agentStateDim+1, 2*agentStateDim) << " " << cov(2*agentStateDim+1, 2*agentStateDim+1);
//      }
      multi_out << "\n";
      ttc.getTopology().write(topo_out, state);
      // simulation
      state = motionModel->move(state, control, motionModel->sampleNoise(state, control));
      if (Random::uniform() < p("multi_rotor_control/kidnap/target").toDouble()) {
        // kidnap target
        state.segment(aSD*nA, 2) = tt.randomJump();
        ekf.getCovariance().block(aSD*nA, aSD*nA, 4, 4) = Eigen::MatrixXd::Identity(4, 4)*4.0;
      } else {
        state.segment(aSD*nA, 2) = tt.step();
      }
      if (Random::uniform() < p("multi_rotor_control/kidnap/agent").toDouble()) {
        // kidnap agent
        int agent = rand()%nA;
        state.segment(aSD*agent, 2) = Eigen::Vector2d(Random::uniform(-1, 1), Random::uniform(-0.5, 0.5));
        ekf.getCovariance().block(aSD*agent, aSD*agent, 2, 2) = Eigen::MatrixXd::Identity(2, 2)*4.0;
      }
      // state estimation
      ekf.predict(control);
      for (std::vector<const SensorModel*>::const_iterator it = sensorModels.begin();
          it != sensorModels.end(); ++it) {
        if ((*it)->measurementAvailable(state)) {
          Eigen::VectorXd noiseSample = (*it)->sampleNoise(state, (*it)->sense(state));
          Eigen::VectorXd measurementSample = (*it)->sense(state, noiseSample);
          ekf.correct(measurementSample, *(*it));
        }
      }
    }
  }

  return 0;
}
