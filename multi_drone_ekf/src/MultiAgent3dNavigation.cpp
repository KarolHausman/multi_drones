#include "multi_drone_ekf/MultiAgent3dNavigation.h"
#include <ranav/ekf.h>
#include <ranav/targettrackingcontroller.h>
#include <ranav/multiagentsensormodel.h>
#include <ranav/multiagentmotionmodel.h>
#include "multi_drone_ekf/marker3dsensormodel.h"
#include "multi_drone_ekf/rotor3dmotionmodel.h"


void MultiAgent3dNavigation::getStateEstimate(std::vector<tf::Transform> &stateEstimate) {
  Eigen::VectorXd mean = ekf->getMean();
  unsigned int nA = motionModel->getNumAgents();
  unsigned int nT = motionModel->getNumTargets();
  unsigned int aSD = motionModel->getAgentStateDim();
  unsigned int tSD = motionModel->getTargetStateDim();
  stateEstimate.resize(nA+nT);
  assert(addOn3d.size() == nA);
  for (unsigned int i=0; i<nA; ++i) {
    assert(aSD == 3);
    Eigen::Vector3d v = mean.segment(i*aSD, 3);
    stateEstimate[i].setOrigin(tf::Vector3(v(0), v(1), addOn3d[i].z));
    tf::Quaternion q;
    q.setRPY(addOn3d[i].roll, addOn3d[i].pitch, v(2));
    stateEstimate[i].setRotation(q);
  }
  assert(nT == 1);
  for (unsigned int i=0; i<nT; ++i) {
    assert(tSD >= 2);
    Eigen::Vector2d v = mean.segment(nA*aSD+i*tSD, 2);
    stateEstimate[nA+i].setOrigin(tf::Vector3(v(0), v(1), 0));
  }
}

void MultiAgent3dNavigation::navigate(const std::vector<Measurement3D> &measurements, const std::vector<Odometry3D> &odometry, std::vector<geometry_msgs::Twist> &control, std::vector<tf::Transform>& stateEstimate)
{
  assert(odometry.size() == addOn3d.size());
  unsigned int nA = motionModel->getNumAgents();
  unsigned int cSD = motionModel->getAgentControlDim();
  assert(cSD == 3);
  assert(odometry.size() == nA);
  Eigen::VectorXd odo(cSD*nA);
  for (std::vector<Odometry3D>::const_iterator it = odometry.begin(); it != odometry.end(); ++it) {
    // update addOn
    double roll, pitch, yaw;
    it->movement.getBasis().getRPY(roll, pitch, yaw);
    addOn3d[it-odometry.begin()] = AddOn3d(roll, pitch, it->movement.getOrigin().getZ());

    // EKF prediction step
    odo.segment(cSD*(it-odometry.begin()), 3) = Eigen::Vector3d(it->movement.getOrigin().getX(), it->movement.getOrigin().getY(), atan2(sin(yaw), cos(yaw)));
  }
  odo /= getCycleDt();
  ekf->predict(odo);

  getStateEstimate(stateEstimate);

  // just take the most recent (last in the vector) measurement for each sensor model
  std::map<ranav::Index, Measurement3D> measurementMap;
  for(std::vector<Measurement3D>::const_iterator m_it = measurements.begin(); m_it != measurements.end(); ++m_it) {
    measurementMap[ranav::Index(m_it->fromId, m_it->toId)] = *m_it;
  }
  for(std::vector<ranav::SensorModel*>::iterator s_it = sensorModels.begin(); s_it != sensorModels.end(); ++s_it) {
    ranav::MultiAgentSensorModel* multiAgentSensorModel = static_cast<ranav::MultiAgentSensorModel*>(*s_it);
    std::map<ranav::Index, Measurement3D>::iterator m_it = measurementMap.find(ranav::Index(multiAgentSensorModel->getFromId(), multiAgentSensorModel->getToId()));
    if (m_it != measurementMap.end()) {
      tf::Transform camPose = world_to_cam;
      if (m_it->second.fromId >= 0) { // one of the agents is sensing
        assert((int)stateEstimate.size() > m_it->second.fromId);
        camPose = stateEstimate.at(m_it->second.fromId) * drone_to_front_cam; // TODO: not valid for multiple drones/agents
      }
      ranav::Marker3dSensorModel* marker3dmodel = static_cast<ranav::Marker3dSensorModel*>(*s_it);
      marker3dmodel->setNoiseCov(camPose, m_it->second.measurement);

      Eigen::VectorXd measurement_2d = marker3dmodel->downProjectMeasurement(m_it->second.measurement, camPose);
      ROS_INFO("Integrating measurement of model (%d, %d) with measurement dim %d", m_it->second.fromId, m_it->second.toId, (int)measurement_2d.size());
      ekf->correct(measurement_2d, *(*s_it));
    }
  }

  getStateEstimate(stateEstimate);

  Eigen::VectorXd controlVec = ttc->getControl(ekf, motionModel, ttc->getTopology().getSensorModels());
  control.resize(nA);
  assert(cSD == 3);
  for (unsigned int i=0; i<nA; ++i) {
    Eigen::Vector3d c = controlVec.segment(cSD*i, 3);
    control[i].linear.x = c(0);
    control[i].linear.y = c(1);
    control[i].angular.z = c(2);
  }

}


MultiAgent3dNavigation::MultiAgent3dNavigation(const tf::Transform& world_to_cam, const tf::Transform& drone_to_marker, const tf::Transform& drone_to_front_cam, const ranav::TParam &p)
{
  this->params = p;
  motionModel = new ranav::Rotor3dMotionModel();
  motionModel->init(p);
  ekf = new ranav::EKF(motionModel);
  ekf->init(p);
  ttc = new ranav::TargetTrackingController();
  ttc->init(p);

  addOn3d.resize(motionModel->getNumAgents());

  this->world_to_cam = world_to_cam;
  this->drone_to_marker = drone_to_marker;
  this->drone_to_front_cam = drone_to_front_cam;

  double roll, pitch, yaw;

  world_to_cam.getBasis().getEulerYPR(yaw, pitch, roll);
  Eigen::Vector3d world_to_cam2d(world_to_cam.getOrigin().getX(),world_to_cam.getOrigin().getY(), yaw);

  drone_to_marker.getBasis().getEulerYPR(yaw, pitch, roll);
  Eigen::Vector3d drone_to_marker2d(drone_to_marker.getOrigin().getX(), drone_to_marker.getOrigin().getY(), yaw);

  drone_to_front_cam.getBasis().getEulerYPR(yaw, pitch, roll);
  Eigen::Vector3d drone_to_front_cam2d(drone_to_front_cam.getOrigin().getX(), drone_to_front_cam.getOrigin().getX(), yaw);

  ranav::AllModels models;
  ranav::Marker3dSensorModel *sm;
  sm = new ranav::Marker3dSensorModel(-1, 0);
  sm->init(p);
  sm->setCameraPose(world_to_cam2d); // HACK: overwrites parameter of config file
  sm->setMarkerPose(drone_to_marker2d); // HACK: overwrites parameter of config file
  models()[ranav::Index(-1, 0)] = sm;
  sm = new ranav::Marker3dSensorModel(0, 1);
  sm->init(p);
  sm->setCameraPose(drone_to_front_cam2d + Eigen::Vector3d(1.0, 0, 0)); // HACK: move virtual 2D camera pose of tilted camera to have a circular field of view around (0, 0)
  models()[ranav::Index(0, 1)] = sm;
  ttc->getTopology().setAllSensorModels(models);
  sensorModels = ttc->getTopology().getSensorModelsNonconst();
}

MultiAgent3dNavigation::~MultiAgent3dNavigation() {
  delete motionModel;
  delete ekf;
  delete ttc;
}
