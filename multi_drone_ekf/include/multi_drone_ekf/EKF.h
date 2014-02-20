/*
 * EKF.h
 *
 *  Created on: Feb 8, 2014
 *      Author: Karol Hausman
 */

#ifndef EKF_H_
#define EKF_H_

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/LU> // for Matrix-Inversion
#include <tf/transform_broadcaster.h>
#include "multi_drone_ekf/motionmodel.h"
#include "multi_drone_ekf/sensormodel.h"




struct ExtendedKalmanFilter {

 Eigen::VectorXd state_; // x, y, yaw
 Eigen::MatrixXd sigma_; // uncertainty of state

 bool initialized_;
 ranav::MotionModel* motion_model_;


 // odometry:
 // x: distance travelled in local x-direction
 // y: distance travelled in local y-direction
 // yaw: rotation update
 void predictionStep(const Eigen::VectorXd& odometry);

 // compare expected and measured values, update state and uncertainty
 void correctionStep(const Eigen::VectorXd& measurement, const ranav::SensorModel &sensorModel);


 void init(const Eigen::VectorXd& mean_init);

 void printState(){
  std::cout << "kalman state: " << state_(0) << "  " << state_(1) << " " << state_(2)/M_PI*180 << std::endl;
 }

 ExtendedKalmanFilter(ranav::MotionModel *m):
     motion_model_(m)
 {
  state_ =  Eigen::Vector3d::Zero();
  sigma_ = Eigen::Matrix3d::Identity();
  initialized_ = false;

 }


 

};


#endif /* EKF_H_ */

