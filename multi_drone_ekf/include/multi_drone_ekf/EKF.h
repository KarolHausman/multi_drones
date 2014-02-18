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



//namespace Eigen
//{
//  typedef Eigen::Matrix<float, 3, 1> Vector3f;
//  typedef Eigen::Matrix<float, 3, 6> Matrix3f;
//}

struct ExtendedKalmanFilter {

 Eigen::Vector3f state_; // x, y, yaw
 Eigen::Matrix3f sigma_; // uncertainty of state

 Eigen::Matrix3f Q_; // process noise
 Eigen::Matrix3f R_; // observation noise
 bool initialized_;

 void predictionStep(const Eigen::Vector3f& odometry); // x_{t+1} = g(x_t,u) and update uncertainty

 void correctionStep(const Eigen::Vector6f& measurement, const tf::Transform& cam_to_world_transform, const tf::Transform& drone_to_marker_transform, const double& roll, const double& pitch, const double& z); // compare expected and measured values, update state and uncertainty

 void reduceMeasurementDimensions (const Eigen::Vector6f& measurement, const tf::Transform& world_to_cam, const tf::Transform& drone_to_marker_transform, Eigen::Vector3f& measurement_3dog);


 void init(const tf::Transform& world_to_drone_pose);

 void printState(){
  std::cout << "kalman state: " << state_(0) << "  " << state_(1) << " " << state_(2)/M_PI*180 << std::endl;
 }

 ExtendedKalmanFilter(){
  state_ =  Eigen::Vector3f::Zero();
  sigma_ = Eigen::Matrix3f::Zero(); sigma_(0,0) = sigma_(1,1) = 1; sigma_(2,2) = 1;

  Q_ = Eigen::Matrix3f::Zero();     Q_(0,0) = 0.02; Q_(1,1) = 0.02; Q_(2,2) = 0.002;
  R_ = Eigen::Matrix3f::Zero();     R_(0,0) = R_(1,1) = 0.001; R_(2,2) = 0.0001;
  initialized_ = false;

 }


 

};


#endif /* EKF_H_ */

