/*
 *
 *  Created on: Feb 20, 2014
 *      Author: Karol Hausman
 */


#ifndef ARDRONECONTROLLER_H_
#define ARDRONECONTROLLER_H_

#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include <math.h>
#include "multi_drone_ekf/PIDcontroller.h"



struct ArdroneController {

    ros::Publisher pub_vel;
    ros::NodeHandle nh_;

    ros::Publisher pub_cmd_marker;

    // new command
    geometry_msgs::Twist twist;

    PIDcontroller pid_x, pid_y, pid_z, pid_yaw;

    float x_goal, y_goal, height_goal, yaw_goal;



    void setParameter(const float& c_proportional, const float& c_integral, const float& c_derivative, const float& yaw_proportional, const float& yaw_integral, const float& yaw_derivative);

    ArdroneController();


    // use this to set a new goal pose for the controller
    void setGoalPose(float x, float y, float height, float yaw);

    // control in xyz and yaw
    void sendNewCommand(const float& position_x, const float& position_y, const float& position_z, const float& position_yaw);
};

#endif /* ARDRONECONTROLLER_H_ */

