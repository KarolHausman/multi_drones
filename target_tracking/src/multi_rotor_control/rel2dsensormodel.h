#ifndef REL2DSENSORMODEL_H_
#define REL2DSENSORMODEL_H_

#include "multiagentsensormodel.h"

namespace ranav {

//! relative observation between two vehicles
class Rel2dSensorModel : public MultiAgentSensorModel {
public:
  Rel2dSensorModel(int fromId, int toId);
  virtual ~Rel2dSensorModel();
  virtual void init(const TParam &p);

  //! returns whether a measurement is available in the given state
  virtual bool measurementAvailable(const Eigen::VectorXd &state) const;

  //! returns the expected measurement given the state of MultiRotor2DMotionModel
  virtual Eigen::VectorXd sense(const Eigen::VectorXd &state, const Eigen::VectorXd &noise) const;
  using SensorModel::sense;

  virtual Eigen::VectorXd sampleNoise(const Eigen::VectorXd &state, const Eigen::VectorXd &measurement) const;
  virtual Eigen::MatrixXd getNoiseCov(const Eigen::VectorXd &state, const Eigen::VectorXd &measurement) const;

  virtual Eigen::MatrixXd jacobianState(const Eigen::VectorXd &state, const Eigen::VectorXd &noise) const;
  virtual Eigen::MatrixXd jacobianNoise(const Eigen::VectorXd &state, const Eigen::VectorXd &noise) const;
  using SensorModel::jacobianState;
  using SensorModel::jacobianNoise;

  int getFromId() const { return fromId; }
  int getToId() const { return toId; }
  virtual double getInformation(const Eigen::VectorXd &state, const Eigen::VectorXd &measurement) const;

protected:
  Eigen::MatrixXd H, W;
  TParam params;
  double measurementNoise;
  double distanceNoiseFactor;
  double visibilityRadius;
};

} /* namespace ranav */

#endif /* REL2DSENSORMODEL_H_ */
