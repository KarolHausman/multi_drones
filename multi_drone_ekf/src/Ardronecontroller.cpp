/*
 *
 *  Created on: Feb 20, 2014
 *      Author: Karol Hausman
 */


#include "multi_drone_ekf/Ardronecontroller.h"
#include <tf/transform_broadcaster.h>





void ArdroneController::setParameter(const float& c_proportional, const float& c_integral, const float& c_derivative, const float& yaw_proportional, const float& yaw_integral, const float& yaw_derivative )
{
    pid_x.c_proportional = pid_y.c_proportional = pid_z.c_proportional =
            c_proportional;
    pid_x.c_integral = pid_y.c_integral = pid_z.c_integral =
            c_integral;
    pid_x.c_derivative = pid_y.c_derivative = pid_z.c_derivative =
            c_derivative;

    pid_yaw.c_proportional = yaw_proportional;
    pid_yaw.c_integral = yaw_integral;
    pid_yaw.c_derivative = yaw_derivative;
}

ArdroneController::ArdroneController()
{
    pub_vel = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    pid_x.c_proportional = pid_y.c_proportional = pid_z.c_proportional = 1.2;
    pid_x.c_integral = pid_y.c_integral = pid_z.c_integral = 1.4;
    pid_x.c_derivative = pid_y.c_derivative = pid_z.c_derivative = 1.6;


    pid_yaw.c_proportional = 0.5;
    pid_yaw.c_integral = 0.7;
    pid_yaw.c_derivative = 0.9;

    twist.linear.x = twist.linear.y = twist.linear.z = 0;
    twist.angular.x = twist.angular.y = twist.angular.z = 0;

}

// use this to set a new goal pose for the controller
void ArdroneController::setGoalPose(float x, float y, float height, float yaw) {
    x_goal = x;
    y_goal = y, height_goal = height;
    yaw_goal = yaw;
}

// control in xyz and yaw
void ArdroneController::sendNewCommand(const float& position_x, const float& position_y, const float& position_z, const float& position_yaw)
{
    ros::Time now = ros::Time::now();

    btMatrix3x3 rotationMatrix;
    rotationMatrix.setEulerYPR(position_yaw, 0, 0);
    btVector3 position;
    position.setX(x_goal - position_x);
    position.setY(y_goal - position_y);
    position.setZ(height_goal - position_z);
    position = position * rotationMatrix;

    std::cout<<"height goal: "<<height_goal<<std::endl;
    twist.linear.x = pid_x.getCommand(position.getX(), now); // velocity_x_in_LOCAL_Frame;
    twist.linear.y = pid_y.getCommand(position.getY(), now); // velocity_y_in_LOCAL_Frame;
    twist.linear.z = pid_z.getCommand(position.getZ(), now);
    twist.angular.z = pid_yaw.getCommand(yaw_goal - position_yaw, now); // normalize angle..

    pub_vel.publish(twist);
}
