/*
 * EKF.h
 *
 *  Created on: Feb 8, 2014
 *      Author: Karol Hausman
 */

#include "multi_drone_ekf/EKF.h"



void ExtendedKalmanFilter::init(const Eigen::VectorXd& mean_init)
{
    state_ = mean_init;
    initialized_ = true;
}



void ExtendedKalmanFilter::predictionStep(const Eigen::VectorXd& odometry)
{
    state_ = motion_model_->move(state_,odometry);
    // dg/dx:
    Eigen::MatrixXd G = Eigen::Matrix3d::Zero();
    G = motion_model_->jacobianState(state_,odometry);

    sigma_ = G * sigma_ * G.transpose() + motion_model_->getNoiseCov(state_,odometry);

}

void ExtendedKalmanFilter::correctionStep(const Eigen::VectorXd& measurement, const ranav::SensorModel &sensorModel)
{

    Eigen::VectorXd h(3);
    h = sensorModel.sense(state_);

    Eigen::MatrixXd dh(3,3);
    dh = sensorModel.jacobianState(state_);

    Eigen::MatrixXd K(3,3);
    Eigen::MatrixXd brackets = dh * sigma_ * dh.transpose() + sensorModel.getNoiseCov(state_, measurement);

    K = sigma_ * dh.transpose() * brackets.inverse();

    sigma_ = (Eigen::Matrix3d::Identity() - K * dh) * sigma_;

    Eigen::VectorXd brackets2 = measurement - h;
    //normalize yaw angle
    brackets2(2) = atan2(sin(brackets2(2)), cos(brackets2(2)));

    state_ = state_ + K * brackets2;
}






