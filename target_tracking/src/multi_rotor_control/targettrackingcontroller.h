#ifndef TARGETTRACKINGCONTROLLER_H_
#define TARGETTRACKINGCONTROLLER_H_

#include "tparam.h"
#include "topology.h"
#include "multiagentmotionmodel.h"

namespace ranav {

class MotionModel;
class SensorModel;
class EKF;

class TargetTrackingController {
public:
  TargetTrackingController();
  virtual ~TargetTrackingController();
  virtual void init(const TParam &p);

  Eigen::VectorXd getControl(const EKF *ekf, const MultiAgentMotionModel *motionModel, const std::vector<const SensorModel*> &sensorModel, double *f = NULL) const;
  Eigen::VectorXd getControlTopo(const EKF *ekf, const MultiAgentMotionModel *motionModel, std::vector<const SensorModel*> &sensorModel);

  const Topology& getTopology() const { return topo; }
  Topology& getTopology() { return topo; }

  struct Evaluator {
    Evaluator(const EKF *e, const MultiAgentMotionModel *m, const std::vector<const SensorModel*> &s, const TParam &p) : ekf(e), motionModel(m), sensorModel(s)
    {
      horizon = p("multi_rotor_control/horizon").toInt();
      nA = p("multi_rotor_control/numAgents").toInt();
      nT = p("multi_rotor_control/numTargets").toInt();
      discountFactor = p("multi_rotor_control/discountFactor").toDouble();
    }
    double evaluate(const Eigen::VectorXd &control) const;
    Eigen::VectorXd gradient(const Eigen::VectorXd &control) const;
    const EKF *ekf;
    const MultiAgentMotionModel *motionModel;
    std::vector<const SensorModel*> sensorModel;
    int horizon;
    int nA; //!< number of agents
    int nT; //!< number of targets
    double discountFactor;
  };

protected:
  TParam params;
  Topology topo;
};

} /* namespace ranav */

#endif /* TARGETTRACKINGCONTROLLER_H_ */
