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
#include "multi_drone_ekf/rotor2Dmotionmodel.h"



namespace Eigen
{
  typedef Eigen::Matrix<float, 6, 1> Vector6f;
  typedef Eigen::Matrix<float, 6, 6> Matrix6f;
}

struct ExtendedKalmanFilter {

 Eigen::VectorXd state_; // x, y, yaw
 Eigen::MatrixXd sigma_; // uncertainty of state

 Eigen::MatrixXd Q_; // process noise
 Eigen::Matrix6f R_prime_; // observation noise 6dog
 Eigen::Matrix3f R_; // observation noise 3dog
 bool initialized_;
 ranav::MotionModel* motion_model_;

 void computeHJacobian(const tf::Transform& cam_to_world_flat, const tf::Transform& drone_to_marker_flat, Eigen::Matrix3f& dh);

 void computeHprimeJacobian (const tf::Transform& world_to_cam_flat, const Eigen::Vector6f& measurement, Eigen::MatrixXf& dh_prime);


 void predictionStep(const Eigen::VectorXd& odometry); // x_{t+1} = g(x_t,u) and update uncertainty

 void correctionStep(const Eigen::Vector6f& measurement, const tf::Transform& cam_to_world_transform, const tf::Transform& drone_to_marker_transform); // compare expected and measured values, update state and uncertainty

 void reduceMeasurementDimensions (const Eigen::Vector6f& measurement, const tf::Transform& world_to_cam, const tf::Transform& marker_to_drone, Eigen::Vector3f& measurement_3dog);


 void init(const tf::Transform& world_to_drone_pose);

 void printState(){
  std::cout << "kalman state: " << state_(0) << "  " << state_(1) << " " << state_(2)/M_PI*180 << std::endl;
 }

 ExtendedKalmanFilter():
     motion_model_(new ranav::Rotor2dMotionModel){
  state_ =  Eigen::Vector3d::Zero();
  sigma_ = Eigen::Matrix3d::Zero(); sigma_(0,0) = sigma_(1,1) = 1; sigma_(2,2) = 1;

  Q_ = Eigen::Matrix3d::Zero();     Q_(0,0) = 0.02; Q_(1,1) = 0.02; Q_(2,2) = 0.002;
  R_prime_ = Eigen::Matrix6f::Zero();     R_prime_(0,0) = R_prime_(1,1) = R_prime_(2,2) = 0.01; R_prime_(3,3) = R_prime_(4,4) = 0.01; R_prime_(5,5) = 0.001;
  initialized_ = false;

 }


 

};


#endif /* EKF_H_ */

