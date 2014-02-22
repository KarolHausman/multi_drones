#ifndef MULTIAGENT3DNAVIGATION_H_
#define MULTIAGENT3DNAVIGATION_H_

#include <ranav/tparam.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>

namespace ranav {
  class MultiAgentMotionModel;
  class EKF;
  class TargetTrackingController;
}

class MultiAgent3dNavigation {
public:
  //! fill TargetTrackingController::Topology::allModels with appropriate Marker3dSensorModels
  MultiAgent3dNavigation();
  ~MultiAgent3dNavigation();

  double getCycleDt() { return params("estimation/motionDt").toDouble(); }

  struct Measurement3D {
    int fromId, toId; //!< agent ID (-1: global GPS; 0, ..., N-1: agents; N: target)
    tf::Transform measurement;
  };
  struct Odometry3D {
    int id; //!< agent ID (0, ..., N-1)
    tf::Transform movement;
  };

  //! downproject all to 2D
  //! apply odometry in EKF
  //! check which measurements have to be used according to current topology and apply them in EKF
  void navigate(const std::vector<Measurement3D> &measurements, const std::vector<Odometry3D> &odometry, std::vector<geometry_msgs::Twist> &control, std::vector<tf::Transform> stateEstimate);

protected:
  // 3D navigation stuff
  tf::Transform world_to_cam;
  tf::Transform drone_to_marker;
  tf::Transform drone_to_front_cam;

  // 2D navigation stuff
  ranav::TParam params;
  ranav::MultiAgentMotionModel *motionModel;
  ranav::EKF *ekf;
  ranav::TargetTrackingController *ttc;
};

#endif
