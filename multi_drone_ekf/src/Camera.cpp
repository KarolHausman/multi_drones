/*
 *
 *  Created on: Feb 17, 2014
 *      Author: Karol Hausman
 */


#include "multi_drone_ekf/Camera.h"

void Camera::tagCB(const multi_drone_ekf::TagsConstPtr& tag_msg, uint marker) {

    int tag_cnt = tag_msg->tag_count;

    if (tag_cnt == 0)
        return;

    for (int i = 0; i < tag_cnt; ++i) {
        if (marker == tag_msg->tags[i].id)
            ROS_INFO(
                        "Found tag  %i (cf: %.3f)", tag_msg->tags[i].id, tag_msg->tags[i].cf);
    }

    for (int i = 0; i < tag_cnt; ++i) {

        multi_drone_ekf::Tag tag = tag_msg->tags[i];



        // detection is too unsure

        if (tag.cf < 0.5)
            continue;


        trans_x_ += tag.xMetric;
        trans_y_ += tag.yMetric;
        trans_z_ += tag.zMetric;
        rot_z_ += -tag.yRot;
        rot_y_ += -tag.xRot;
        rot_x_ += tag.zRot;



        btVector3 trans_around_y(0,0,0);

        btQuaternion rot_around_y;
        rot_around_y.setEulerZYX(0,-M_PI/2,0);

        tf::Transform pose_around_y;


        pose_around_y.setOrigin(trans_around_y);
        pose_around_y.setRotation(rot_around_y);

        counter_ ++;


        if (counter_ == avg_number_)
        {

            btVector3 translation(trans_x_/avg_number_, trans_y_/avg_number_, trans_z_/avg_number_);


            btQuaternion rotation;

            rotation.setEulerZYX(rot_z_/avg_number_, rot_y_/avg_number_, rot_x_/avg_number_);


            if (tag.id == marker){
                tag_pose_.setOrigin(translation);
                tag_pose_.setRotation(rotation);
                tag_pose_ = tag_pose_*pose_around_y;
                pose_set_ = true;
            }
        }
    }
}


Camera::Camera(uint marker_nr) {

    pose_set_ = false;
    trans_x_ = 0; trans_y_ = 0; trans_z_ = 0;
    rot_x_ = 0; rot_y_ = 0; rot_z_ = 0;
    counter_ = 0;
    avg_number_ = 10;
    boost::function<void (const multi_drone_ekf::TagsConstPtr&)> tag_callback( boost::bind(&Camera::tagCB, this, _1, marker_nr) );
    sub_tags_ = nh_.subscribe("/tags", 100,  tag_callback);

}



