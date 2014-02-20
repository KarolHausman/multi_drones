#include "multi_drone_ekf/marker2dsensormodel.h"
#include <Eigen/Cholesky>
#include <iostream>
#include "multi_drone_ekf/random.h"

namespace ranav {

Marker2dSensorModel::Marker2dSensorModel(const tf::Transform& cam_to_world_flat, const tf::Transform& drone_to_marker_flat)
{
    this->cam_to_world_flat = cam_to_world_flat;
    this->drone_to_marker_flat = drone_to_marker_flat;
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

Marker2dSensorModel::~Marker2dSensorModel() {
}


bool Marker2dSensorModel::measurementAvailable(const Eigen::VectorXd &state) const {
    return true;
}

Eigen::VectorXd Marker2dSensorModel::sense(const Eigen::VectorXd &state, const Eigen::VectorXd &noise) const {

    tf::Transform state_pose_flat;
    btVector3 state_origin_flat(state(0),state(1),0);
    state_pose_flat.setOrigin(state_origin_flat);
    btQuaternion state_quaternion_flat;
    state_quaternion_flat.setEulerZYX(state(2),0,0);
    state_pose_flat.setRotation(state_quaternion_flat);



    tf::Transform h_transform_flat;

    h_transform_flat = cam_to_world_flat*state_pose_flat*drone_to_marker_flat;

    Eigen::VectorXd h(3);
    h << h_transform_flat.getOrigin().getX(), h_transform_flat.getOrigin().getY(), tf::getYaw(h_transform_flat.getRotation());

    return h;



}

Eigen::VectorXd Marker2dSensorModel::sampleNoise(const Eigen::VectorXd &state, const Eigen::VectorXd &measurement) const {
  return Random::multivariateGaussian(getNoiseCov(state, measurement));
}

Eigen::MatrixXd Marker2dSensorModel::getNoiseCov(const Eigen::VectorXd &state, const Eigen::VectorXd &measurement) const {
    return noiseCov;
}

double Marker2dSensorModel::getInformation(const Eigen::VectorXd &state, const Eigen::VectorXd &measurement) const {
    return 0;
}

Eigen::MatrixXd Marker2dSensorModel::jacobianState(const Eigen::VectorXd &state, const Eigen::VectorXd &) const
{
    double c11,c12,/*c13,*/c21,c22,/*c23,*/m11,/*m12,*/m13,m21,/*m22,*/m23;
    tf::Matrix3x3 c_rot = cam_to_world_flat.getBasis();
//    tf::Vector3 c_transl = cam_to_world_flat.getOrigin();

    c11 = c_rot.getRow(0).getX(); c12 = c_rot.getRow(0).getY(); //c13 = c_transl.getX();
    c21 = c_rot.getRow(1).getX(); c22 = c_rot.getRow(1).getY(); //c23 = c_transl.getY();


    tf::Matrix3x3 m_rot = drone_to_marker_flat.getBasis();
    tf::Vector3 m_transl = drone_to_marker_flat.getOrigin();
    m11 = m_rot.getRow(0).getX();/* m12 = m_rot.getRow(0).getY();*/ m13 = m_transl.getX();
    m21 = m_rot.getRow(1).getX();/* m22 = m_rot.getRow(1).getY();*/ m23 = m_transl.getY();

    Eigen::MatrixXd dh(3,3);


    dh << c11,c12,m13*(c12*cos(state(2)) - c11*sin(state(2))) +
             m23*(-(c11*cos(state(2))) - c12*sin(state(2))),
           c21,c22,m13*(c22*cos(state(2)) - c21*sin(state(2))) +
             m23*(-(c21*cos(state(2))) - c22*sin(state(2))),
           0,0,((m11*(c22*cos(state(2)) - c21*sin(state(2))) +
                  m21*(-(c21*cos(state(2))) - c22*sin(state(2))))/
                (m21*(c12*cos(state(2)) - c11*sin(state(2))) + m11*(c11*cos(state(2)) + c12*sin(state(2)))) -
               ((m11*(c12*cos(state(2)) - c11*sin(state(2))) + m21*(-(c11*cos(state(2))) - c12*sin(state(2))))*
                  (m21*(c22*cos(state(2)) - c21*sin(state(2))) + m11*(c21*cos(state(2)) + c22*sin(state(2)))))/
                pow(m21*(c12*cos(state(2)) - c11*sin(state(2))) + m11*(c11*cos(state(2)) + c12*sin(state(2))),
                 2))/(1 + pow(m21*(c22*cos(state(2)) - c21*sin(state(2))) +
                  m11*(c21*cos(state(2)) + c22*sin(state(2))),2)/
                pow(m21*(c12*cos(state(2)) - c11*sin(state(2))) + m11*(c11*cos(state(2)) + c12*sin(state(2))),
                 2));
    return dh;


}

Eigen::MatrixXd Marker2dSensorModel::jacobianNoise(const Eigen::VectorXd &, const Eigen::VectorXd &) const {
  return W;
}

} /* namespace ranav */

