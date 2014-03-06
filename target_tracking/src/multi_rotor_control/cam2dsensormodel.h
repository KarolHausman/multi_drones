#ifndef CAM2DSENSORMODEL_H_
#define CAM2DSENSORMODEL_H_

#include "multiagentsensormodel.h"
#include <Eigen/Geometry>

namespace ranav {

//! relative observation between two vehicles
class Cam2dSensorModel : public MultiAgentSensorModel {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Cam2dSensorModel(int fromId, int toId);
  virtual ~Cam2dSensorModel();
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

  virtual double getInformation(const Eigen::VectorXd &state, const Eigen::VectorXd &measurement) const;

  static Eigen::Isometry2d fromVec(const Eigen::Vector3d &v);
  static Eigen::Vector3d toVec(const Eigen::Isometry2d &i);

  // TODO: to be overwritten in AR.Drone code
  void setCameraPose(const Eigen::Vector3d &p) { cameraPose = p; }
  void setMarkerPose(const Eigen::Vector3d &p) { markerPose = p; }
  void setVisibilityRadius(double r) { visibilityRadius = r; }

protected:
  Eigen::MatrixXd H, W;
  int fromIdx, toIdx;
  TParam params;
  double measurementNoise;
  double distanceNoiseFactor;
  double visibilityRadius;
  Eigen::Vector3d cameraPose; //!< camera pose (in the frame of reference of fromId)
  Eigen::Vector3d markerPose; //!< marker pose (in the frame of reference of toId)
};

} /* namespace ranav */

#endif /* CAM2DSENSORMODEL_H_ */
