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


    state_(3)=odometry(3);
    state_(4)=odometry(4);
    state_(5)=odometry(5);


    btQuaternion newRotation;
    newRotation.setEulerZYX(odometry(3), odometry(4), odometry(5));
    state_pose_.setRotation(newRotation);

    //Transform(rotate) local translation vector to global translation
    btVector3 translation;
    translation.setX(odometry(0));
    translation.setY(odometry(1));
    translation.setZ(0);

    btMatrix3x3 rotationMatrix;
    rotationMatrix.setEulerYPR(odometry(3), odometry(4), odometry(5));
    translation = rotationMatrix * translation;
    translation.setZ(odometry(2)-state_(2));

    state_(0) = state_pose_.getOrigin().getX()+translation.getX();
    state_(1) = state_pose_.getOrigin().getY()+translation.getY();
    state_(2) = state_pose_.getOrigin().getZ()+translation.getZ();



    //Integrate/Sum translation values
//    btVector3 newOrigin(odom_pose_.getOrigin().getX() + translation.getX(), odom_pose_.getOrigin().getY() + translation.getY(), odom_pose_.getOrigin().getZ() + translation.getZ()); //holds integrated state
    btVector3 newOrigin(state_(0),state_(1),state_(2));

    state_pose_.setOrigin(newOrigin);



//	state(0) = state(0) + cos(state(2)) * odometry(0)
//			- sin(state(2)) * odometry(1);
//	state(1) = state(1) + sin(state(2)) * odometry(0)
//			+ cos(state(2)) * odometry(1);
//	state(2) = state(2) + odometry(2);

//	state(2) = atan2(sin(state(2)), cos(state(2))); // normalize angle

//	// dg/dx:
    Eigen::Matrix6f G;
    double a = state_(1);
    double b = state_(2);
    double c = state_(3);
    double z = state_(4);//yaw
    double y = state_(5);//pitch
    double x = state_(6);//roll

    double G11 = cos(x)*cos(y);
    double G12 = -sin(x)*cos(y);
    double G13 = 0;
    double G14 = 0;
    double G15 = sin(y)*(b*sin(x)-a*cos(x));
    double G16 = -cos(y)*(a*sin(x)+b*cos(x));

    double G21 = cos(x)*sin(y)*sin(z) + sin(x)*cos(z);
    double G22 = cos(x)*cos(z)-sin(x)*sin(y)*sin(z);
    double G23 = 0;
    double G24 = cos(x)*(a*sin(y)*cos(z)-b*sin(z)) - sin(x)*(a*sin(z)+b*sin(y)*cos(z));
    double G25 = cos(y)*sin(z)*(a*cos(x)-b*sin(x));
    double G26 = cos(x)*(a*cos(z)-b*sin(y)*sin(z))-sin(x)*(a*sin(y)*sin(z)+b*cos(z));

    double G31,G32,G33,G34,G35,G36;
    G31 = G32 = G33 = G34 = G35 = G36 =0;
    G33 = 1;
    double G41,G42,G43,G44,G45,G46;
    G41 = G42 = G43 = G44 = G45 = G46 =0;
    G44 = 1;
    double G51,G52,G53,G54,G55,G56;
    G51 = G52 = G53 = G54 = G55 = G56 =0;
    G55 = 1;
    double G61,G62,G63,G64,G65,G66;
    G61 = G62 = G63 = G64 = G65 = G66 =0;
    G66 = 1;

    G<< G11,G12,G13,G14,G15,G16, G21,G22,G23,G24,G25,G26, G31,G32,G33,G34,G35,G36,G41,G42,G43,G44,G45,G46,G51,G52,G53,G54,G55,G56,G61,G62,G63,G64,G65,G66;

//	G << 1, 0, -sin(state(2)) * odometry(0) - cos(state(2)) * odometry(1), 0, 1, cos(
//			state(2)) * odometry(0) - sin(state(2)) * odometry(1), 0, 0, 1;

     std::cout << "G: " << std::endl << G << std::endl;

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



