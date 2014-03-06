#ifndef MULTIAGENTMOTIONMODEL_H_
#define MULTIAGENTMOTIONMODEL_H_

#include "motionmodel.h"

namespace ranav {

class MultiAgentMotionModel : public MotionModel {
public:
  MultiAgentMotionModel() : nA(0), nT(0), agentStateDim(0), targetStateDim(0), agentControlDim(0) {}
  virtual ~MultiAgentMotionModel() {}

  unsigned int getNumAgents() const { return nA; }
  unsigned int getNumTargets() const { return nT; }
  unsigned int getAgentStateDim() const { return agentStateDim; }
  unsigned int getTargetStateDim() const { return targetStateDim; }
  unsigned int getAgentControlDim() const { return agentControlDim; }

protected:
  unsigned int nA; //!< number of agents
  unsigned int nT; //!< number of targets
  unsigned int agentStateDim; //!< dimensionality of the state of a single agent
  unsigned int targetStateDim; //!< dimensionality of the state of a single target
  unsigned int agentControlDim; //!< dimensionality of the control of a single agent
};

} /* namespace ranav */

#endif /* MULTIAGENTMOTIONMODEL_H_ */
