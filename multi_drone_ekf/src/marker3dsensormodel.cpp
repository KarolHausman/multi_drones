#include "multi_drone_ekf/marker3dsensormodel.h"
#include <Eigen/Cholesky>
#include <iostream>
//#include "multi_drone_ekf/random.h"

namespace ranav {

Marker3dSensorModel::Marker3dSensorModel(const tf::Transform& cam_to_world, const tf::Transform& drone_to_marker):
    Marker2dSensorModel(cam_to_world,drone_to_marker)
{
    noiseCov = Eigen::MatrixXd::Identity(3,3);
    noiseCovSqrt = Eigen::MatrixXd::Identity(3,3);
    stateDim = 3;
    measurementDim = 3;
    noiseDim = 2;
    H = Eigen::MatrixXd::Zero(3, 3);
    W = Eigen::MatrixXd::Identity(3, 3);
    visibilityRadius = 0.2;
    measurementNoise = 0.01;
    distanceNoiseFactor = 0.2;
}

Marker3dSensorModel::~Marker3dSensorModel()
{
}


Eigen::VectorXd Marker3dSensorModel::downProjectMeasurement(const Eigen::VectorXd& measurement)
{

}

void Marker3dSensorModel::setNoiseCov(const Eigen::MatrixXd& noise_6dog)
{

}

};
