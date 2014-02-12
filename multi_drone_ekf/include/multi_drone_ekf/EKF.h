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



namespace Eigen
{
  typedef Eigen::Matrix<float, 6, 1> Vector6f;
  typedef Eigen::Matrix<float, 6, 6> Matrix6f;
}

struct ExtendedKalmanFilter {

 Eigen::Vector6f state_; // x, y, z, roll, pitch, yaw
 Eigen::Matrix6f sigma_; // uncertainty of state

 Eigen::Matrix6f Q_; // process noise
 Eigen::Matrix6f R_; // observation noise
 tf::Transform state_pose_;

 void predictionStep(const Eigen::Vector6f& odometry); // x_{t+1} = g(x_t,u) and update uncertainty
 void correctionStep(const Eigen::Vector6f& measurement);  // compare expected and measured values, update state and uncertainty

 void printState(){
  std::cout << "kalman state: " << state_(0) << "  " << state_(1) << " " << state_(2) << " " << state_(3)/M_PI*180 << " " << state_(4)/M_PI*180 << " " << state_(5)/M_PI*180 << std::endl;
 }

 ExtendedKalmanFilter(){
  state_ =  Eigen::Vector6f::Zero();
  sigma_ = Eigen::Matrix6f::Zero(); sigma_(0,0) = sigma_(1,1) = 1; sigma_(2,2) = sigma_(3,3) = 1;
  sigma_(4,4) = sigma_(5,5) = 1;
  Q_ = Eigen::Matrix6f::Zero();     Q_(0,0) = 0.004; Q_(1,1) = 0.002; Q_(2,2) = 0.00002; Q_(3,3) = 0.00002;
  Q_(4,4) = 0.00002; Q_(5,5) = 0.00002;
  R_ = Eigen::Matrix6f::Zero();     R_(0,0) = R_(1,1) = 0.01; R_(2,2) = 0.0001; R_(3,3) = 0.0001;
  R_(4,4) = 0.0001; R_(5,5) = 0.0001;
 }


 

};


#endif /* EKF_H_ */

