/*
 *
 *  Created on: Feb 17, 2014
 *      Author: Karol Hausman
 */


#include "multi_drone_ekf/Camera.h"

void Camera::tagCB(const ar_pose::ARMarkers::ConstPtr& tag_msg, uint marker) {

    int tag_cnt = tag_msg->markers.size();

    if (tag_cnt == 0)
        return;

    for (int i = 0; i < tag_cnt; ++i) {
        ROS_DEBUG("Found tag  %i (cf: %i) for ID = %i", tag_msg->markers[i].id, tag_msg->markers[i].confidence, marker);

        ar_pose::ARMarker tag = tag_msg->markers[i];

        // detection is too unsure
        if (tag.confidence < 0.5)
            continue;


        if(tag.id == marker)
        {
            counter_ ++;


            if (counter_ > 100)
            {
                trans_x_ += tag.pose.pose.position.x;
                trans_y_ += tag.pose.pose.position.y;
                trans_z_ += tag.pose.pose.position.z;
                rot_x_ += tag.pose.pose.orientation.x;
                rot_y_ += tag.pose.pose.orientation.y;
                rot_z_ += tag.pose.pose.orientation.z;
                rot_w_ += tag.pose.pose.orientation.w;

                 if (counter_== (100 + avg_number_))
                 {

                    btVector3 translation(trans_x_/avg_number_, trans_y_/avg_number_, trans_z_/avg_number_);

                    btQuaternion rotation(rot_x_/avg_number_, rot_y_/avg_number_, rot_z_/avg_number_, rot_w_/avg_number_);


                    tag_pose_.setOrigin(translation);
                    tag_pose_.setRotation(rotation);
//                    tag_pose_ = tag_pose_*pose_around_y_;
                    pose_set_ = true;

                }
            }
        }
    }
}


Camera::Camera(uint marker_nr) {

    pose_set_ = false;
    trans_x_ = 0; trans_y_ = 0; trans_z_ = 0;
    rot_x_ = 0; rot_y_ = 0; rot_z_ = 0;
    counter_ = 0;
    avg_number_ = 30;
    boost::function<void (const ar_pose::ARMarkers::ConstPtr&)> tag_callback( boost::bind(&Camera::tagCB, this, _1, marker_nr) );
    sub_tags_ = nh_.subscribe("node100/tags", 100,  tag_callback);

    btVector3 trans_around_y(0,0,0);

    btQuaternion rot_around_y;
    rot_around_y.setEulerZYX(0,-M_PI/2,0);

    pose_around_y_.setOrigin(trans_around_y);
    pose_around_y_.setRotation(rot_around_y);

}



