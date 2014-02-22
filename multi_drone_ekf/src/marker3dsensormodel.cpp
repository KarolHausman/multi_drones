#include "multi_drone_ekf/marker3dsensormodel.h"
#include <Eigen/Cholesky>
#include <iostream>
//#include "multi_drone_ekf/random.h"

namespace ranav {

Marker3dSensorModel::Marker3dSensorModel(int from, int to) : Cam2dSensorModel(from, to)
{
    noiseCovPrime = Eigen::MatrixXd::Zero(6,6);
    noiseCovPrime(0,0) = noiseCovPrime(1,1) = noiseCovPrime(2,2) = 0.01;
    noiseCovPrime(3,3) = noiseCovPrime(4,4) = 0.01;
    noiseCovPrime(5,5) = 0.001;
}

Marker3dSensorModel::~Marker3dSensorModel()
{
}


Eigen::VectorXd Marker3dSensorModel::downProjectMeasurement(const tf::Transform& measurement, const tf::Transform& world_to_cam) const
{
    double c_yaw = 0;
    double c_pitch = 0;
    double c_roll = 0;

    world_to_cam.getBasis().getEulerYPR(c_yaw,c_pitch,c_roll);

    double c_x =0;double c_y=0; //double c_z=0;

    c_x = world_to_cam.getOrigin().getX();
    c_y = world_to_cam.getOrigin().getY();
//    c_z = world_to_cam.getOrigin().getZ();

//    tf::Transform cam_to_marker;
//    tf::Vector3 origin(measurement(0), measurement(1), measurement(2));
//    cam_to_marker.setOrigin(origin);
//    tf::Quaternion rotation;
//    rotation.setRPY(measurement(3),measurement(4), measurement(5));
//    cam_to_marker.setRotation(rotation);

    tf::Transform world_to_marker = world_to_cam * measurement;

    double m_yaw = 0;
    double m_pitch = 0;
    double m_roll = 0;

    world_to_marker.getBasis().getEulerYPR(m_yaw,m_pitch,m_roll);

    double m_x =0;double m_y=0; //double m_z=0;

    m_x = world_to_marker.getOrigin().getX();
    m_y = world_to_marker.getOrigin().getY();
//    m_z = world_to_marker.getOrigin().getZ();

    // 2D camera position wrt. world or agent
    tf::Quaternion c_2d_rotation;
    c_2d_rotation.setRPY(0,0,c_yaw);
    tf::Transform c_2d(c_2d_rotation, tf::Vector3(c_x,c_y,0));

    tf::Quaternion m_2d_rotation;
    m_2d_rotation.setRPY(0,0,m_yaw);
    tf::Transform m_2d(m_2d_rotation, tf::Vector3(m_x,m_y,0));

    tf::Transform z_2d = c_2d.inverse()*m_2d;

    Eigen::VectorXd measurement_3dog = Eigen::Vector3d::Zero();

    measurement_3dog(0) = z_2d.getOrigin().getX();
    measurement_3dog(1) = z_2d.getOrigin().getY();
    measurement_3dog(2) = tf::getYaw(z_2d.getRotation());

    return measurement_3dog;
}

void Marker3dSensorModel::setNoiseCov(const tf::Transform& world_to_cam, const tf::Transform& measurement)
{

    double r = 0;
    double p = 0;
    double y = 0;
    measurement.getBasis().getEulerYPR(y,p,r);

    //pitch and roll of the measurement
//    p = measurement(4);
//    y = measurement(5);

    double c11,c12,c13,/*c14,*/c21,c22,c23/*,c24,c31,c32,c33,c34*/;
    tf::Matrix3x3 c_rot = world_to_cam.getBasis();
//    tf::Vector3 c_transl = cam_to_world_flat.getOrigin();

    c11 = c_rot.getRow(0).getX(); c12 = c_rot.getRow(0).getY(); c13 = c_rot.getRow(0).getZ(); //c14 = c_transl.getX();
    c21 = c_rot.getRow(1).getX(); c22 = c_rot.getRow(1).getY(); c23 = c_rot.getRow(1).getZ(); //c24 = c_transl.getY();
    //c31 = c_rot.getRow(2).getX(); c32 = c_rot.getRow(2).getY(); c33 = c_rot.getRow(2).getZ(); c34 = c_transl.getZ();

    Eigen::MatrixXd dh_prime(3,6);

    dh_prime << c11*cos(atan2(c21,c11)) + c21*sin(atan2(c21,c11)),
            c12*cos(atan2(c21,c11)) + c22*sin(atan2(c21,c11)),
            c13*cos(atan2(c21,c11)) + c23*sin(atan2(c21,c11)),0,0,0,
           c21*cos(atan2(c21,c11)) - c11*sin(atan2(c21,c11)),
            c22*cos(atan2(c21,c11)) - c12*sin(atan2(c21,c11)),
            c23*cos(atan2(c21,c11)) - c13*sin(atan2(c21,c11)),0,0,0,
           0,0,0,0,((c13*c21 - c11*c23)*cos(y) + (c13*c22 - c12*c23)*sin(y))/
             ((pow(c13,2) + pow(c23,2))*pow(sin(p),2) -
               sin(2*p)*(c11*c13*cos(y) + c21*c23*cos(y) + c12*c13*sin(y) +
                  c22*c23*sin(y)) + pow(cos(p),2)*
                ((pow(c11,2) + pow(c21,2))*pow(cos(y),2) +
                  (pow(c12,2) + pow(c22,2))*pow(sin(y),2) +
                  (c11*c12 + c21*c22)*sin(2*y))),
            (cos(p)*((-(c12*c21) + c11*c22)*cos(p) +
                 sin(p)*((-(c13*c22) + c12*c23)*cos(y) + (c13*c21 - c11*c23)*sin(y))
                 ))/((pow(c13,2) + pow(c23,2))*pow(sin(p),2) -
               sin(2*p)*(c11*c13*cos(y) + c21*c23*cos(y) + c12*c13*sin(y) +
                  c22*c23*sin(y)) + pow(cos(p),2)*
                ((pow(c11,2) + pow(c21,2))*pow(cos(y),2) +
                  (pow(c12,2) + pow(c22,2))*pow(sin(y),2) +
                  (c11*c12 + c21*c22)*sin(2*y)));

    noiseCov = dh_prime * noiseCovPrime * dh_prime.transpose();
}

};
