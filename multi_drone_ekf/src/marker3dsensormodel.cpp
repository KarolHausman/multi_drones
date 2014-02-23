#include "multi_drone_ekf/marker3dsensormodel.h"
#include <Eigen/Cholesky>
#include <iostream>
//#include "multi_drone_ekf/random.h"

namespace ranav {

Marker3dSensorModel::Marker3dSensorModel(int from, int to) : Cam2dSensorModel(from, to)
{
    noiseCovPrime = Eigen::MatrixXd::Zero(6,6);
    noiseCovPrime(0,0) = 0.1;
    noiseCovPrime(1,1) = noiseCovPrime(2,2) = 0.01;
    noiseCovPrime(3,3) = noiseCovPrime(4,4) = 0.01;
    noiseCovPrime(5,5) = 0.001;
}

Marker3dSensorModel::~Marker3dSensorModel()
{
}


Eigen::VectorXd Marker3dSensorModel::downProjectMeasurement(const tf::Transform& measurement, const tf::Transform& world_to_cam) const
{
  double roll, pitch, yaw;
  tf::Quaternion q;

  tf::Transform world_to_marker = world_to_cam * measurement;

    // 2D camera position wrt. world or agent
//    world_to_cam.getBasis().getEulerYPR(yaw, pitch, roll);
//    q.setRPY(0,0,yaw);
//    tf::Transform c_2d(q, tf::Vector3(world_to_cam.getOrigin().getX(), world_to_cam.getOrigin().getY(), 0));
    // HACK: using 2D camera pose that is not the 3D pose for having a circular field of view of tilted cameras by shifting the virtual 2D camera position
  q.setRPY(0, 0, cameraPose(2));
  tf::Transform world_to_cam2d(q, tf::Vector3(cameraPose(0), cameraPose(1), 0));

  world_to_marker.getBasis().getEulerYPR(yaw, pitch, roll);
  q.setRPY(0,0,yaw);
  tf::Transform world_to_marker2d(q, tf::Vector3(world_to_marker.getOrigin().getX(), world_to_marker.getOrigin().getY(), 0));

  tf::Transform projected_measurement = world_to_cam2d.inverse()*world_to_marker2d;
  Eigen::VectorXd mproj = Eigen::Vector3d(projected_measurement.getOrigin().getX(),
                         projected_measurement.getOrigin().getY(),
                         tf::getYaw(projected_measurement.getRotation()));
  assert(measurementDim <= 3);
  mproj.conservativeResize(measurementDim);
  return mproj;
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

    Eigen::MatrixXd projnoiseCov = dh_prime * noiseCovPrime * dh_prime.transpose();
    assert(noiseDim <= 3);
    projnoiseCov.conservativeResize(noiseDim, noiseDim);
    noiseCov = projnoiseCov;
}

};
