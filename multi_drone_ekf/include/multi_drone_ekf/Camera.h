/*
 *
 *  Created on: Feb 17, 2014
 *      Author: Karol Hausman
 */


#ifndef CAMERA_H_
#define CAMERA_H_

#include "multi_drone_ekf/Tag.h"
#include "multi_drone_ekf/Tags.h"
#include "ros/ros.h"
#include <tf/transform_broadcaster.h>



struct Camera {

    ros::NodeHandle nh_;
    ros::Subscriber sub_tags_;
    tf::Transform tag_pose_;
    int counter_;
    double trans_x_, trans_y_, trans_z_;
    double rot_x_, rot_y_, rot_z_;
    int avg_number_;
    bool pose_set_;



    void tagCB(const multi_drone_ekf::TagsConstPtr& tag_msg, uint marker);

    Camera(uint marker_nr);

};
#endif /* CAMERA_H_ */
