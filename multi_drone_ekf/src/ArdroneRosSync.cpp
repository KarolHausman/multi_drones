#include "multi_drone_ekf/ArdroneRosSync.h"
#include "multi_drone_ekf/MultiAgent3dNavigation.h"


ArdroneRosSync::ArdroneRosSync(ros::NodeHandle &nh, MultiAgent3dNavigation *navigation) :
navigation(navigation)
{
  assert(navigation != NULL);
  cycleDt = navigation->getCycleDt();
    //create agent map
    Agent agent;
    agent.markerId = 1; // TODO: parameter
    agent.ardroneId = 1; // TODO: parameter
    agents[agent.ardroneId] = agent;
    globalId = 100; // TODO: parameter
    targetMarkerId = 9; // TODO: parameter

    //transform for the tagCB
    btVector3 trans_around_y(0,0,0);
    btQuaternion rot_around_y;
    rot_around_y.setEulerZYX(0,-M_PI/2,0);
    pose_around_y.setOrigin(trans_around_y);
    pose_around_y.setRotation(rot_around_y);



    { // subscribe to global camera
      boost::function<void (const multi_drone_ekf::TagsConstPtr&)> tag_callback( boost::bind(&ArdroneRosSync::tagCB, this, _1, globalId) );
      std::stringstream ss;
      ss << globalId;
      std::string tag_topic = "/node" + ss.str() + "/tags";
      sub_tags.push_back(nh.subscribe(tag_topic, 100, tag_callback));
    }
    // subscribe to all agent's cameras and create control publishers
    std::map<int, Agent>::iterator iter;
    for (iter = agents.begin(); iter != agents.end(); ++iter)
    {
        boost::function<void (const multi_drone_ekf::TagsConstPtr&)> tag_callback( boost::bind(&ArdroneRosSync::tagCB, this, _1, iter->second.ardroneId) );
        std::stringstream ss;
        ss << iter->second.ardroneId;
        std::string tag_topic = "/node" + ss.str() + "/tags";
        sub_tags.push_back(nh.subscribe(tag_topic, 100, tag_callback));

        boost::function<void (const multi_drone_ekf::NavdataConstPtr&)> nav_callback( boost::bind(&ArdroneRosSync::navCB, this, _1, iter->second.ardroneId) );
        std::string nav_topic = "/node" + ss.str() + "/navdata";
        sub_navs.push_back(nh.subscribe(nav_topic, 100, nav_callback));

        iter->second.pub_control = nh.advertise<geometry_msgs::Twist>("/node" + ss.str() + "/cmd_vel", 1);
    }
}

ArdroneRosSync::~ArdroneRosSync() {

}



void ArdroneRosSync::tagCB(const multi_drone_ekf::TagsConstPtr& tag_msg, int ardroneId)
{
    int tag_cnt = tag_msg->tag_count;

    if (tag_cnt == 0)
        return;

    for (int i = 0; i < tag_cnt; ++i) {
        ROS_INFO("Found tag  %i (cf: %.3f) for ID = %i", tag_msg->tags[i].id, tag_msg->tags[i].cf, ardroneId);

        multi_drone_ekf::Tag tag = tag_msg->tags[i];

        // detection is too unsure
        if (tag.cf < 0.5)
            continue;

        double trans_x_ = tag.xMetric;
        double trans_y_ = tag.yMetric;
        double trans_z_ = tag.zMetric;
        double rot_z_ = -tag.yRot;
        double rot_y_ = -tag.xRot;
        double rot_x_ = tag.zRot;




        btVector3 translation(trans_x_,trans_y_,trans_z_);
        btQuaternion rotation;
        rotation.setEulerZYX(rot_z_, rot_y_,rot_x_);

        tf::Transform tag_pose;
        tag_pose.setOrigin(translation);
        tag_pose.setRotation(rotation);
        tag_pose = tag_pose*pose_around_y;

        if (ardroneId == globalId) {
          globalMeasurements.push_back(std::make_pair(tag.id, tag_pose));
        } else {
          agents[ardroneId].measurements.push_back(std::make_pair(tag.id, tag_pose));
        }
    }

}


void ArdroneRosSync::navCB(const multi_drone_ekf::NavdataConstPtr& nav_msg, int ardroneId)
{

    ROS_DEBUG_STREAM(
                "------------------------------------------ \n"
                << "Nav_msg vx: " << nav_msg->vx
                << "\nNav_msg vy: " << nav_msg->vy
                << "\nNav_msg vz: " << nav_msg->vz
                << "\nNav_msg yaw: " << nav_msg->rotZ
                << "\nNav_msg height: " << nav_msg->altd);



    double x = nav_msg->vx / 1000;
    double y = nav_msg->vy / 1000;
    double z = nav_msg->altd / 1000;
    double roll = nav_msg->rotX / (180.0 / M_PI);
    double pitch = nav_msg->rotY / (180.0 / M_PI);
    double yaw = nav_msg->rotZ / (180.0 / M_PI);


    btVector3 odom_trans(x, y, z);
    btQuaternion odom_rot;
    odom_rot.setEulerZYX(yaw, pitch, roll);

    tf::Transform odometry;
    odometry.setOrigin(odom_trans);
    odometry.setRotation(odom_rot);

    agents[ardroneId].odometry = odometry;

    checkCycle();
}


void ArdroneRosSync::checkCycle() {
  // check for first cycle
  if (!lastCycle.isValid()) {
    bool gotOdo = true;
    for (std::map<int, Agent>::iterator it = agents.begin();
        it != agents.end(); ++it) {
      if (!it->second.gotOdo)
        gotOdo = false;
    }
    if (!gotOdo) {
      return;
    }
    // last timestep initialization for incremental odometry
    lastCycle = ros::Time::now();
    for (std::map<int, Agent>::iterator it = agents.begin();
        it != agents.end(); ++it) {
      it->second.last_odometry = it->second.odometry;
    }
    return;
  }

  // check if cycleDt passed
  ros::Time now = ros::Time::now();
  if ((now - lastCycle).toSec() < cycleDt) {
    return;
  }
  lastCycle = now;

  assert(agents.size() == 1);

  // compute incremental odometry, set last_odometry
  std::vector<MultiAgent3dNavigation::Odometry3D> odometry;
  MultiAgent3dNavigation::Odometry3D odo;
  odo.id = 0;
  for (std::map<int, Agent>::iterator it = agents.begin();
      it != agents.end(); ++it, ++odo.id) {
    // odometry transform contains: visual_odometry_x, visual_odometry_y, sonar_height_z, imu_roll, imu_pitch, visual_odometry_yaw
    double z = it->second.odometry.getOrigin().getZ();
    double roll, pitch, yaw;
    it->second.last_odometry.getBasis().getEulerYPR(yaw, pitch, roll);
    tf::Quaternion q;
    q.setRPY(0, 0, yaw);
    tf::Transform lastOdo2d(q, tf::Vector3(it->second.last_odometry.getOrigin().getX(), it->second.last_odometry.getOrigin().getY(), 0));
    it->second.odometry.getBasis().getEulerYPR(yaw, pitch, roll);
    q.setRPY(yaw, 0, 0);
    tf::Transform odo2d(q, tf::Vector3(it->second.odometry.getOrigin().getX(), it->second.odometry.getOrigin().getY(), 0));
    odo.movement = lastOdo2d.inverse() * odo2d;
    q.setRPY(roll, pitch, tf::getYaw(odo.movement.getRotation()));
    odo.movement.setRotation(q);
    odo.movement.getOrigin().setZ(z);
    it->second.last_odometry = it->second.odometry;
    odometry.push_back(odo);
  }

  // replace marker ids by sensed object ids (also include target)
  std::vector<MultiAgent3dNavigation::Measurement3D> measurements;
  MultiAgent3dNavigation::Measurement3D measurement;
  measurement.fromId = 0;
  for (std::map<int, Agent>::iterator it = agents.begin();
      it != agents.end(); ++it, ++measurement.fromId) {
    for (std::vector<std::pair<int, tf::Transform> >::iterator m = it->second.measurements.begin();
        m != it->second.measurements.end(); ++m) {
      if (m->first == targetMarkerId) {
        measurement.toId = agents.size(); // target has ID N (for N agents)
        measurement.measurement = m->second;
        measurements.push_back(measurement);
      }
      measurement.toId = 0;
      for (std::map<int, Agent>::iterator ag = agents.begin();
          ag != agents.end(); ++ag, ++measurement.toId) {
        if (m->first == ag->second.markerId) {
          measurement.measurement = m->second;
          measurements.push_back(measurement);
        }
      }
    }
    it->second.measurements.clear();
  }



  measurement.fromId = -1;
  for (std::vector<std::pair<int, tf::Transform> >::iterator m = globalMeasurements.begin();
      m != globalMeasurements.end(); ++m) {
    if (m->first == targetMarkerId) {
      measurement.toId = agents.size(); // target has ID N (for N agents)
      measurement.measurement = m->second;
      measurements.push_back(measurement);
    }
    measurement.toId = 0;
    for (std::map<int, Agent>::iterator ag = agents.begin();
        ag != agents.end(); ++ag, ++measurement.toId) {
      if (m->first == ag->second.markerId) {
        measurement.measurement = m->second;
        measurements.push_back(measurement);
      }
    }
  }

  globalMeasurements.clear();


  std::cout << "-------------------ODOMETRY: ---------------------------------" << std::endl;
  for(std::vector<MultiAgent3dNavigation::Odometry3D>::iterator o_it = odometry.begin(); o_it != odometry.end(); ++o_it)
  {
      std::cout << "Agent ID= " << o_it->id << std::endl;
      std::cout << "Agent Transform: " << std::endl;
      std::cout <<"x: " << o_it->movement.getOrigin().getX() << std::endl;
      std::cout <<"y: " << o_it->movement.getOrigin().getY() << std::endl;
      std::cout <<"z: " << o_it->movement.getOrigin().getZ() << std::endl;
      double roll, pitch, yaw;
      o_it->movement.getBasis().getRPY(roll, pitch, yaw);
      std::cout <<"roll: " << roll << std::endl;
      std::cout <<"pitch: " << pitch << std::endl;
      std::cout <<"yaw: " << yaw << std::endl;

  }


  std::cout << "-------------------MEASUREMENTS: -------------------------------" <<std::endl;
  std::cout << "MEASUREMENTS SIZE: " << measurements.size() << std::endl;
  for(std::vector<MultiAgent3dNavigation::Measurement3D>::iterator m_it = measurements.begin(); m_it != measurements.end(); ++m_it)
  {
      std::cout << std::endl << "From ID= " << m_it->fromId << std::endl;
      std::cout << "To ID= " << m_it->toId << std::endl;
      std::cout << "Measurement Transform: " << std::endl;
      std::cout <<"x: " << m_it->measurement.getOrigin().getX() << std::endl;
      std::cout <<"y: " << m_it->measurement.getOrigin().getY() << std::endl;
      std::cout <<"z: " << m_it->measurement.getOrigin().getZ() << std::endl;
      double roll, pitch, yaw;
      m_it->measurement.getBasis().getRPY(roll, pitch, yaw);
      std::cout <<"roll: " << roll << std::endl;
      std::cout <<"pitch: " << pitch << std::endl;
      std::cout <<"yaw: " << yaw << std::endl;

  }

//  // call navigation function
//  std::vector<geometry_msgs::Twist> control;
//  std::vector<tf::Transform> stateEstimate;
//  navigation->navigate(measurements, odometry, control, stateEstimate);

//  // publish state estimate
//  assert(stateEstimate.size() == agents.size()+1);
//  int i = 0;
//  for (std::map<int, Agent>::iterator it = agents.begin();
//      it != agents.end(); ++it, ++i) {
//    std::stringstream ss;
//    ss << "/node" << it->second.ardroneId;
//    ROS_INFO("Transform called");
//    transform_broadcaster.sendTransform(tf::StampedTransform(stateEstimate[i], now, "/world", ss.str()));
//  }

//  // publish controls (not in first test)
//  assert(control.size() == agents.size());
//  i = 0;
//  for (std::map<int, Agent>::iterator it = agents.begin();
//      it != agents.end(); ++it, ++i) {
////    it->second.pub_control.publish(control[i]);
//  }
}
