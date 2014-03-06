#ifndef MULTIAGENTSENSORMODEL_H_
#define MULTIAGENTSENSORMODEL_H_

#include "sensormodel.h"

namespace ranav {

//! relative observation between two vehicles
class MultiAgentSensorModel : public SensorModel {
public:
  virtual ~MultiAgentSensorModel() {}

  int getFromId() const { return fromId; }
  int getToId() const { return toId; }
  virtual double getInformation(const Eigen::VectorXd &state, const Eigen::VectorXd &measurement) const = 0;

protected:
  int fromId, toId;
  unsigned int nA; //!< num agents
  unsigned int nT; //!< num targets
};

} /* namespace ranav */

#endif /* MULTIAGENTSENSORMODEL_H_ */
