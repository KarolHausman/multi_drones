/*
 * EKF.h
 *
 *  Created on: Feb 8, 2014
 *      Author: Karol Hausman
 */

#include "multi_drone_ekf/EKF.h"



// odometry:
// x: distance travelled in local x-direction
// y: distance travelled in local y-direction
// phi: rotation update
void ExtendedKalmanFilter::predictionStep(const Eigen::Vector6f& odometry) {

//	state(0) = state(0) + cos(state(2)) * odometry(0)
//			- sin(state(2)) * odometry(1);
//	state(1) = state(1) + sin(state(2)) * odometry(0)
//			+ cos(state(2)) * odometry(1);
//	state(2) = state(2) + odometry(2);

//	state(2) = atan2(sin(state(2)), cos(state(2))); // normalize angle

//	// dg/dx:
//	Eigen::Matrix3f G;

//	G << 1, 0, -sin(state(2)) * odometry(0) - cos(state(2)) * odometry(1), 0, 1, cos(
//			state(2)) * odometry(0) - sin(state(2)) * odometry(1), 0, 0, 1;

//	// cout << "G: " << endl << G << endl;

//	sigma = G * sigma * G.transpose() + Q;

}

void ExtendedKalmanFilter::correctionStep(const Eigen::Vector6f& measurement) { // compare expected and measured values, update state and uncertainty

//	Eigen::Vector3f h;

//	h(0) = cos(state(2)) * (global_marker_pose(0) - state(0))
//			- sin(state(2)) * (global_marker_pose(1) - state(1)); //x
//	h(1) = sin(state(2)) * (global_marker_pose(0) - state(0))
//			+ cos(state(2)) * (global_marker_pose(1) - state(1)); //y
//	h(2) = global_marker_pose(2) - state(2); //yaw

//	Eigen::Matrix3f dh;
//	Eigen::Matrix3f K;

//	dh << -cos(state(2)), sin(state(2)), -sin(state(2)) * (global_marker_pose(0) - state(0)) + cos(state(2)) * (global_marker_pose(1) - state(1)),
//			-sin(state(2)), -cos(state(2)), cos(state(2)) * (global_marker_pose(0) - state(0)) - sin(state(2)) * (global_marker_pose(1) - state(1)),
//			0, 0, -1;

//	Eigen::Matrix3f brackets = dh * sigma * dh.transpose() + R;

//	K = sigma * dh.transpose() * brackets.inverse();

//	sigma = (Matrix3f::Identity() - K * dh) * sigma;

//	Eigen::Vector3f brackets2 = measurement - h;
//	//normalize yaw angle
//	brackets2(2) = atan2(sin(brackets2(2)), cos(brackets2(2)));

//	state = state + K * brackets2;
}



