#ifndef MARKER2DSENSORMODEL_H_
#define MARKER2DSENSORMODEL_H_

#include "multi_drone_ekf/sensormodel.h"
#include <tf/transform_broadcaster.h>
#include <ros/ros.h>



namespace ranav {

class Marker2dSensorModel : public SensorModel {
public:
  Marker2dSensorModel(const tf::Transform& cam_to_world_flat, const tf::Transform& drone_to_marker_flat);

  virtual ~Marker2dSensorModel();
//  virtual void init(const TParam &p);

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

  double getInformation(const Eigen::VectorXd &state, const Eigen::VectorXd &measurement) const;

protected:
  Eigen::MatrixXd H, W;
//  TParam params;
//  unsigned int nA; //!< num agents
//  unsigned int nT; //!< num targets
  double measurementNoise;
  double distanceNoiseFactor;
  double visibilityRadius;
  tf::Transform cam_to_world_flat;
  tf::Transform drone_to_marker_flat;
};

} /* namespace ranav */

#endif /* MARKER2DSENSORMODEL_H_ */

