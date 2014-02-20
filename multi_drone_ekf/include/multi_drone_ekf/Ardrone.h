/*
 *
 *  Created on: Feb 17, 2014
 *      Author: Karol Hausman
 */


#ifndef ARDRONE_H_
#define ARDRONE_H_

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <tf/transform_broadcaster.h>
#include "multi_drone_ekf/EKF.h"


#include "multi_drone_ekf/Navdata.h"
#include "multi_drone_ekf/Tag.h"
#include "multi_drone_ekf/Tags.h"
#include <Eigen/Core>
#include <boost/bind.hpp>
#include "multi_drone_ekf/rotor3Dmotionmodel.h"



struct Ardrone {

	ros::NodeHandle nh_;
    ros::Subscriber sub_nav_;
    ros::Subscriber sub_tags_;
    ExtendedKalmanFilter* kalman_filter_;
    ranav::Rotor3dMotionModel motionModel;


    tf::Transform tag_pose_;
    tf::Transform drone_in_marker_coord_;

    bool tag_seen_first_time_;
    bool navCB_done_;
    tf::Transform odom_pose_;
    double prevTime_;
    double last_yaw_, last_roll_, last_pitch_;
    bool initialized_;
    double distZ_;
    tf::Transform state_pose_;
    tf::Transform world_to_drone_pose_;
    tf::Transform world_to_cam_transform_;

    double yaw_;
    double pitch_;
    double roll_;




    void tagCB(const multi_drone_ekf::TagsConstPtr& tag_msg, uint marker);

    void navCB(const multi_drone_ekf::NavdataConstPtr& nav_msg);

    Ardrone(const uint& marker_nr);

};

#endif /* ARDRONE_H_ */





