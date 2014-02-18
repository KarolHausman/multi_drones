/*
 * EKF.h
 *
 *  Created on: Feb 8, 2014
 *      Author: Karol Hausman
 */

#include "multi_drone_ekf/EKF.h"


void ExtendedKalmanFilter::init(const tf::Transform& world_to_drone_pose)
{
//    state_(0) = world_to_drone_pose.getOrigin().getX();
//    state_(1) = world_to_drone_pose.getOrigin().getY();
//    double yaw = 0;
//    double pitch = 0;
//    double roll = 0;
//    world_to_drone_pose.getBasis().getEulerYPR(yaw, pitch, roll);
//    state_(2) = yaw;
    initialized_ = true;

}



// odometry:
// x: distance travelled in local x-direction
// y: distance travelled in local y-direction
// yaw: rotation update
void ExtendedKalmanFilter::predictionStep(const Eigen::Vector3f& odometry) {




    state_(0) = state_(0) + cos(state_(2)) * odometry(0)
            - sin(state_(2)) * odometry(1);
    state_(1) = state_(1) + sin(state_(2)) * odometry(0)
            + cos(state_(2)) * odometry(1);
    state_(2) = state_(2) + odometry(2);

    state_(2) = atan2(sin(state_(2)), cos(state_(2))); // normalize angle

    // dg/dx:
    Eigen::Matrix3f G;


    G << 1, 0, -sin(state_(2)) * odometry(0) - cos(state_(2)) * odometry(1), 0, 1, cos(
            state_(2)) * odometry(0) - sin(state_(2)) * odometry(1), 0, 0, 1;


//     std::cout << "G: " << std::endl << G << std::endl;

    sigma_ = G * sigma_ * G.transpose() + Q_;

}

void ExtendedKalmanFilter::correctionStep(const Eigen::Vector6f& measurement, const tf::Transform& cam_to_world_transform, const tf::Transform& drone_to_marker_transform, const double& roll, const double& pitch, const double& z) { // compare expected and measured values, update state and uncertainty

    Eigen::Vector3f h;

    tf::Transform state_pose;
    btVector3 state_origin(state_(0),state_(1),z);
    state_pose.setOrigin(state_origin);
    btQuaternion state_quaternion;
    state_quaternion.setEulerZYX(state_(2),pitch,roll);
    state_pose.setRotation(state_quaternion);


    tf::Transform H_transform;
    H_transform = cam_to_world_transform*state_pose*drone_to_marker_transform;

    double h_yaw = 0;
    double h_pitch = 0;
    double h_roll = 0;

    H_transform.getBasis().getEulerYPR(h_yaw, h_pitch, h_roll);

    h << H_transform.getOrigin().getX(),H_transform.getOrigin().getY(),h_yaw;



    tf::Matrix3x3 c_rot = cam_to_world_transform.getBasis();
//    tf::Vector3 c_transl = cam_to_world_transform.getOrigin();

    double c11,c12,/*c13,*/c21,c22,/*c23,*/c31,c32/*,c33,c41,c42,c43*/;

    c11 = c_rot.getRow(0).getX(); c21 = c_rot.getRow(0).getY(); c31 = c_rot.getRow(0).getZ();
    c12 = c_rot.getRow(1).getX(); c22 = c_rot.getRow(1).getY(); c32 = c_rot.getRow(1).getZ();
//    c13 = c_rot.getRow(2).getX(); c23 = c_rot.getRow(2).getY(); c33 = c_rot.getRow(2).getZ();

//    c41 = c_transl.getX();
//    c42 = c_transl.getY();
//    c43 = c_transl.getZ();


    tf::Matrix3x3 m_rot = drone_to_marker_transform.getBasis();
    tf::Vector3 m_transl = drone_to_marker_transform.getOrigin();

    double m11,m12,m13,/*m21,m22,m23,m31,m32,m33.*/m41,m42,m43;

    m11 = m_rot.getRow(0).getX(); /*m21 = m_rot.getRow(0).getY(); m31 = m_rot.getRow(0).getZ();*/
    m12 = m_rot.getRow(1).getX(); /*m22 = m_rot.getRow(1).getY(); m32 = m_rot.getRow(1).getZ();*/
    m13 = m_rot.getRow(2).getX(); /*m23 = m_rot.getRow(2).getY(); m33 = m_rot.getRow(2).getZ();*/

    m41 = m_transl.getX();
    m42 = m_transl.getY();
    m43 = m_transl.getZ();

    Eigen::Matrix3f dh;
    Eigen::Matrix3f K;

//   dh<< c11,c21,m41*(c21*cos(pitch)*cos(state_(2)) - c11*cos(pitch)*sin(state_(2))) +
//           m43*(c21*(cos(roll)*cos(state_(2))*sin(pitch) + sin(roll)*sin(state_(2))) + c11*(cos(state_(2))*sin(roll) - cos(roll)*sin(pitch)*sin(state_(2)))) +
//           m42*(c21*(cos(state_(2))*sin(roll)*sin(pitch) - cos(roll)*sin(state_(2))) + c11*(-(cos(roll)*cos(state_(2))) - sin(roll)*sin(pitch)*sin(state_(2)))),
//         c12,c22,m41*(c22*cos(pitch)*cos(state_(2)) - c12*cos(pitch)*sin(state_(2))) +
//           m43*(c22*(cos(roll)*cos(state_(2))*sin(pitch) + sin(roll)*sin(state_(2))) + c12*(cos(state_(2))*sin(roll) - cos(roll)*sin(pitch)*sin(state_(2)))) +
//           m42*(c22*(cos(state_(2))*sin(roll)*sin(pitch) - cos(roll)*sin(state_(2))) + c12*(-(cos(roll)*cos(state_(2))) - sin(roll)*sin(pitch)*sin(state_(2)))),
//         0,0,(-(((m31*(c23*cos(pitch)*cos(state_(2)) - c13*cos(pitch)*sin(state_(2))) +
//                    m33*(c23*(cos(roll)*cos(state_(2))*sin(pitch) + sin(roll)*sin(state_(2))) + c13*(cos(state_(2))*sin(roll) - cos(roll)*sin(pitch)*sin(state_(2)))) +
//                    m32*(c23*(cos(state_(2))*sin(roll)*sin(pitch) - cos(roll)*sin(state_(2))) + c13*(-(cos(roll)*cos(state_(2))) - sin(roll)*sin(pitch)*sin(state_(2)))))*
//                  (m21*(c13*cos(pitch)*cos(state_(2)) - c33*sin(pitch) + c23*cos(pitch)*sin(state_(2))) +
//                    m23*(c33*cos(roll)*cos(pitch) + c13*(cos(roll)*cos(state_(2))*sin(pitch) + sin(roll)*sin(state_(2))) + c23*(-(cos(state_(2))*sin(roll)) + cos(roll)*sin(pitch)*sin(state_(2)))) +
//                    m22*(c33*cos(pitch)*sin(roll) + c13*(cos(state_(2))*sin(roll)*sin(pitch) - cos(roll)*sin(state_(2))) + c23*(cos(roll)*cos(state_(2)) + sin(roll)*sin(pitch)*sin(state_(2))))))/
//                pow(m31*(c13*cos(pitch)*cos(state_(2)) - c33*sin(pitch) + c23*cos(pitch)*sin(state_(2))) +
//                  m33*(c33*cos(roll)*cos(pitch) + c13*(cos(roll)*cos(state_(2))*sin(pitch) + sin(roll)*sin(state_(2))) + c23*(-(cos(state_(2))*sin(roll)) + cos(roll)*sin(pitch)*sin(state_(2)))) +
//                  m32*(c33*cos(pitch)*sin(roll) + c13*(cos(state_(2))*sin(roll)*sin(pitch) - cos(roll)*sin(state_(2))) + c23*(cos(roll)*cos(state_(2)) + sin(roll)*sin(pitch)*sin(state_(2)))),2)) +
//             (m21*(c23*cos(pitch)*cos(state_(2)) - c13*cos(pitch)*sin(state_(2))) + m23*(c23*(cos(roll)*cos(state_(2))*sin(pitch) + sin(roll)*sin(state_(2))) +
//                   c13*(cos(state_(2))*sin(roll) - cos(roll)*sin(pitch)*sin(state_(2)))) +
//                m22*(c23*(cos(state_(2))*sin(roll)*sin(pitch) - cos(roll)*sin(state_(2))) + c13*(-(cos(roll)*cos(state_(2))) - sin(roll)*sin(pitch)*sin(state_(2)))))/
//              (m31*(c13*cos(pitch)*cos(state_(2)) - c33*sin(pitch) + c23*cos(pitch)*sin(state_(2))) +
//                m33*(c33*cos(roll)*cos(pitch) + c13*(cos(roll)*cos(state_(2))*sin(pitch) + sin(roll)*sin(state_(2))) + c23*(-(cos(state_(2))*sin(roll)) + cos(roll)*sin(pitch)*sin(state_(2)))) +
//                m32*(c33*cos(pitch)*sin(roll) + c13*(cos(state_(2))*sin(roll)*sin(pitch) - cos(roll)*sin(state_(2))) + c23*(cos(roll)*cos(state_(2)) + sin(roll)*sin(pitch)*sin(state_(2))))))/
//           (1 + pow(m21*(c13*cos(pitch)*cos(state_(2)) - c33*sin(pitch) + c23*cos(pitch)*sin(state_(2))) +
//                m23*(c33*cos(roll)*cos(pitch) + c13*(cos(roll)*cos(state_(2))*sin(pitch) + sin(roll)*sin(state_(2))) + c23*(-(cos(state_(2))*sin(roll)) + cos(roll)*sin(pitch)*sin(state_(2)))) +
//                m22*(c33*cos(pitch)*sin(roll) + c13*(cos(state_(2))*sin(roll)*sin(pitch) - cos(roll)*sin(state_(2))) + c23*(cos(roll)*cos(state_(2)) + sin(roll)*sin(pitch)*sin(state_(2)))),2)/
//              pow(m31*(c13*cos(pitch)*cos(state_(2)) - c33*sin(pitch) + c23*cos(pitch)*sin(state_(2))) +
//                m33*(c33*cos(roll)*cos(pitch) + c13*(cos(roll)*cos(state_(2))*sin(pitch) + sin(roll)*sin(state_(2))) + c23*(-(cos(state_(2))*sin(roll)) + cos(roll)*sin(pitch)*sin(state_(2)))) +
//                m32*(c33*cos(pitch)*sin(roll) + c13*(cos(state_(2))*sin(roll)*sin(pitch) - cos(roll)*sin(state_(2))) + c23*(cos(roll)*cos(state_(2)) + sin(roll)*sin(pitch)*sin(state_(2)))),2));

dh<< c11,c21,m41*cos(pitch)*(c21*cos(state_(2)) - c11*sin(state_(2))) +
               sin(roll)*(c11*m43*cos(state_(2)) + c21*m42*cos(state_(2))*sin(pitch) + c21*m43*sin(state_(2)) -
                  c11*m42*sin(pitch)*sin(state_(2))) -
               cos(roll)*(cos(state_(2))*(c11*m42 - c21*m43*sin(pitch)) +
                  (c21*m42 + c11*m43*sin(pitch))*sin(state_(2))),
             c12,c22,m41*cos(pitch)*(c22*cos(state_(2)) - c12*sin(state_(2))) +
               sin(roll)*(c12*m43*cos(state_(2)) + c22*m42*cos(state_(2))*sin(pitch) + c22*m43*sin(state_(2)) -
                  c12*m42*sin(pitch)*sin(state_(2))) -
               cos(roll)*(cos(state_(2))*(c12*m42 - c22*m43*sin(pitch)) +
                  (c22*m42 + c12*m43*sin(pitch))*sin(state_(2))),
             0,0,((-(c22*m13*cos(state_(2))*sin(roll)) - c32*m11*sin(pitch) +
                    c12*m12*cos(state_(2))*sin(roll)*sin(pitch) + c12*m13*sin(roll)*sin(state_(2)) +
                    c22*m12*sin(roll)*sin(pitch)*sin(state_(2)) +
                    cos(pitch)*(c12*m11*cos(state_(2)) + c32*m12*sin(roll) + c22*m11*sin(state_(2))) +
                    cos(roll)*(c32*m13*cos(pitch) + c22*m12*cos(state_(2)) + c12*m13*cos(state_(2))*sin(pitch) -
                       c12*m12*sin(state_(2)) + c22*m13*sin(pitch)*sin(state_(2))))*
                  (cos(pitch)*(-(c21*m11*cos(state_(2))) + c11*m11*sin(state_(2))) -
                    sin(roll)*(c11*m13*cos(state_(2)) + c21*m12*cos(state_(2))*sin(pitch) + c21*m13*sin(state_(2)) -
                       c11*m12*sin(pitch)*sin(state_(2))) +
                    cos(roll)*(cos(state_(2))*(c11*m12 - c21*m13*sin(pitch)) +
                       (c21*m12 + c11*m13*sin(pitch))*sin(state_(2)))) +
                 (-(c21*m13*cos(state_(2))*sin(roll)) - c31*m11*sin(pitch) +
                    c11*m12*cos(state_(2))*sin(roll)*sin(pitch) + c11*m13*sin(roll)*sin(state_(2)) +
                    c21*m12*sin(roll)*sin(pitch)*sin(state_(2)) +
                    cos(pitch)*(c11*m11*cos(state_(2)) + c31*m12*sin(roll) + c21*m11*sin(state_(2))) +
                    cos(roll)*(c31*m13*cos(pitch) + c21*m12*cos(state_(2)) + c11*m13*cos(state_(2))*sin(pitch) -
                       c11*m12*sin(state_(2)) + c21*m13*sin(pitch)*sin(state_(2))))*
                  (m11*cos(pitch)*(c22*cos(state_(2)) - c12*sin(state_(2))) +
                    sin(roll)*(c12*m13*cos(state_(2)) + c22*m12*cos(state_(2))*sin(pitch) + c22*m13*sin(state_(2)) -
                       c12*m12*sin(pitch)*sin(state_(2))) -
                    cos(roll)*(cos(state_(2))*(c12*m12 - c22*m13*sin(pitch)) +
                       (c22*m12 + c12*m13*sin(pitch))*sin(state_(2)))))/
               (pow(-(c21*m13*cos(state_(2))*sin(roll)) - c31*m11*sin(pitch) +
                   c11*m12*cos(state_(2))*sin(roll)*sin(pitch) + c11*m13*sin(roll)*sin(state_(2)) +
                   c21*m12*sin(roll)*sin(pitch)*sin(state_(2)) +
                   cos(pitch)*(c11*m11*cos(state_(2)) + c31*m12*sin(roll) + c21*m11*sin(state_(2))) +
                   cos(roll)*(c31*m13*cos(pitch) + c21*m12*cos(state_(2)) + c11*m13*cos(state_(2))*sin(pitch) -
                      c11*m12*sin(state_(2)) + c21*m13*sin(pitch)*sin(state_(2))),2)*
                 (1 + pow(-(c22*m13*cos(state_(2))*sin(roll)) - c32*m11*sin(pitch) +
                      c12*m12*cos(state_(2))*sin(roll)*sin(pitch) + c12*m13*sin(roll)*sin(state_(2)) +
                      c22*m12*sin(roll)*sin(pitch)*sin(state_(2)) +
                      cos(pitch)*(c12*m11*cos(state_(2)) + c32*m12*sin(roll) + c22*m11*sin(state_(2))) +
                      cos(roll)*(c32*m13*cos(pitch) + c22*m12*cos(state_(2)) +
                         c12*m13*cos(state_(2))*sin(pitch) - c12*m12*sin(state_(2)) + c22*m13*sin(pitch)*sin(state_(2))
                         ),2)/
                    pow(-(c21*m13*cos(state_(2))*sin(roll)) - c31*m11*sin(pitch) +
                      c11*m12*cos(state_(2))*sin(roll)*sin(pitch) + c11*m13*sin(roll)*sin(state_(2)) +
                      c21*m12*sin(roll)*sin(pitch)*sin(state_(2)) +
                      cos(pitch)*(c11*m11*cos(state_(2)) + c31*m12*sin(roll) + c21*m11*sin(state_(2))) +
                      cos(roll)*(c31*m13*cos(pitch) + c21*m12*cos(state_(2)) +
                         c11*m13*cos(state_(2))*sin(pitch) - c11*m12*sin(state_(2)) + c21*m13*sin(pitch)*sin(state_(2))
                         ),2)));





    Eigen::Matrix3f brackets = dh * sigma_ * dh.transpose() + R_;

    K = sigma_ * dh.transpose() * brackets.inverse();

    sigma_ = (Eigen::Matrix3f::Identity() - K * dh) * sigma_;

    Eigen::Vector3f measurement_3dog = Eigen::Vector3f::Zero();

    measurement_3dog(0) = measurement(0);
    measurement_3dog(1) = measurement(1);
    measurement_3dog(2) = measurement(5);
//    reduceMeasurementDimensions(measurement, cam_to_world_transform.inverse(), drone_to_marker_transform.inverse(), measurement_3dog);

    Eigen::Vector3f brackets2 = measurement_3dog - h;
    //normalize yaw angle
    brackets2(2) = atan2(sin(brackets2(2)), cos(brackets2(2)));

//    std::cerr<<"measurement - h: "<<brackets2<<std::endl;

    std::cout<<"h: "<<h<<std::endl;

//    std::cout<<"K: "<<K<<std::endl;



//    printState();
    state_ = state_ + K * brackets2;
//    printState();


}

void ExtendedKalmanFilter::reduceMeasurementDimensions (const Eigen::Vector6f& measurement, const tf::Transform& world_to_cam, const tf::Transform& marker_to_drone, Eigen::Vector3f& measurement_3dog)
{
    double c_yaw = 0;
    double c_pitch = 0;
    double c_roll = 0;

    world_to_cam.getBasis().getEulerYPR(c_yaw,c_pitch,c_roll);

    double c_x =0;double c_y=0; //double c_z=0;

    c_x = world_to_cam.getOrigin().getX();
    c_y = world_to_cam.getOrigin().getY();
//    c_z = world_to_cam.getOrigin().getZ();


    tf::Transform cam_to_marker;
    tf::Vector3 origin(measurement(0), measurement(1), measurement(2));
    cam_to_marker.setOrigin(origin);
    tf::Quaternion rotation;
    rotation.setRPY(measurement(3),measurement(4), measurement(5));
    cam_to_marker.setRotation(rotation);

    tf::Transform world_to_marker = world_to_cam * cam_to_marker;

    double m_yaw = 0;
    double m_pitch = 0;
    double m_roll = 0;

    world_to_marker.getBasis().getEulerYPR(m_yaw,m_pitch,m_roll);

    double m_x =0;double m_y=0; //double m_z=0;

    m_x = world_to_marker.getOrigin().getX();
    m_y = world_to_marker.getOrigin().getY();
//    m_z = world_to_marker.getOrigin().getZ();

    Eigen::Matrix3d c_3d;
    Eigen::Matrix3d z_3d;
    Eigen::Matrix3d m_3d;

    c_3d << cos(c_yaw),-sin(c_yaw),c_x,
            sin(c_yaw),cos(c_yaw),c_y,
            0,0,1;


    m_3d << cos(m_yaw),-sin(m_yaw),m_x,
            sin(m_yaw),cos(m_yaw),m_y,
            0,0,1;

    z_3d = c_3d.inverse()*m_3d;

    measurement_3dog(0) = z_3d(0,2);
    measurement_3dog(1) = z_3d(1,2);
    measurement_3dog(2) = atan2(z_3d(1,0),z_3d(0,0));




}



