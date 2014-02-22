#include "multi_drone_ekf/MultiAgent3dNavigation.h"
#include <ranav/ekf.h>
#include <ranav/multiagentsensormodel.h>
#include "multi_drone_ekf/marker3dsensormodel.h"


void MultiAgent3dNavigation::navigate(const std::vector<Measurement3D> &measurements, const std::vector<Odometry3D> &odometry, std::vector<geometry_msgs::Twist> &control, std::vector<tf::Transform> stateEstimate)
{

    for(std::vector<Measurement3D>::const_iterator m_it = measurements.begin(); m_it != measurements.end(); ++m_it)
    {
        for(std::vector<ranav::SensorModel*>::iterator s_it = sensorModels.begin(); s_it != sensorModels.end(); ++s_it)
        {

            ranav::MultiAgentSensorModel* multiAgentSensorModel = static_cast<ranav::MultiAgentSensorModel*>(*s_it);

            if((multiAgentSensorModel->getFromId() == m_it->fromId)
                && (multiAgentSensorModel->getToId() == m_it->toId))
            {

                ranav::Marker3dSensorModel* marker3dmodel = static_cast<ranav::Marker3dSensorModel*>(*s_it);
                marker3dmodel->setNoiseCov(world_to_cam, m_it->measurement);

                Eigen::VectorXd measurement_2d = Eigen::Vector3d::Zero();
                measurement_2d = marker3dmodel->downProjectMeasurement(m_it->measurement, world_to_cam);
                ekf->correct(measurement_2d, *(*s_it));
            }

        }
    }


}


MultiAgent3dNavigation::MultiAgent3dNavigation(const tf::Transform& world_to_cam, const tf::Transform& drone_to_marker, const tf::Transform& drone_to_front_cam)
{
    this->world_to_cam = world_to_cam;
    this->drone_to_marker = drone_to_marker;
    this->drone_to_front_cam = drone_to_front_cam;

    tf::Transform world_to_cam2d;
    double c_yaw = 0;
    double c_pitch = 0;
    double c_roll = 0;
    world_to_cam.getBasis().getEulerYPR(c_yaw,c_pitch,c_roll);
    btVector3 c2w_origin_flat(world_to_cam.getOrigin().getX(),world_to_cam.getOrigin().getY(),0);
    world_to_cam2d.setOrigin(c2w_origin_flat);
    btQuaternion c2w_quaternion_flat;
    c2w_quaternion_flat.setEulerZYX(c_yaw,0,0);
    world_to_cam2d.setRotation(c2w_quaternion_flat);
    cam_to_world2d = world_to_cam2d.inverse();


    double d_yaw = 0;
    double d_pitch = 0;
    double d_roll = 0;
    drone_to_marker.getBasis().getEulerYPR(d_yaw, d_pitch, d_roll);
    btVector3 d2m_origin_flat(drone_to_marker.getOrigin().getX(),drone_to_marker.getOrigin().getY(),0);
    drone_to_marker2d.setOrigin(d2m_origin_flat);
    btQuaternion d2m_quaternion_flat;
    d2m_quaternion_flat.setEulerZYX(d_yaw,0,0);
    drone_to_marker2d.setRotation(d2m_quaternion_flat);
}
