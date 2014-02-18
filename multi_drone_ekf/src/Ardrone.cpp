/*
 *
 *  Created on: Feb 17, 2014
 *      Author: Karol Hausman
 */


#include "multi_drone_ekf/Ardrone.h"


void Ardrone::tagCB(const multi_drone_ekf::TagsConstPtr& tag_msg, uint marker) {

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





        double trans_x_ = tag.xMetric;
        double trans_y_ = tag.yMetric;
        double trans_z_ = tag.zMetric;
        double rot_z_ = -tag.yRot;
        double rot_y_ = -tag.xRot;
        double rot_x_ = tag.zRot;



        btVector3 trans_around_y(0,0,0);

        btQuaternion rot_around_y;
        rot_around_y.setEulerZYX(0,-M_PI/2,0);

        tf::Transform pose_around_y;



        pose_around_y.setOrigin(trans_around_y);
        pose_around_y.setRotation(rot_around_y);

        btVector3 translation(trans_x_,trans_y_,trans_z_);
        btQuaternion rotation;
        rotation.setEulerZYX(rot_z_, rot_y_,rot_x_);


        if (tag.id==marker){


            tag_pose_.setOrigin(translation);
            tag_pose_.setRotation(rotation);
            tag_pose_ = tag_pose_*pose_around_y;

            tag_seen_first_time_ = true;

            if(initialized_)
            {
                Eigen::Vector6f measurement;
                measurement(0) = tag_pose_.getOrigin().getX();
                measurement(1) = tag_pose_.getOrigin().getY();
                measurement(2) = tag_pose_.getOrigin().getZ();

                double roll = 0;
                double pitch = 0;
                double yaw = 0;
                tag_pose_.getBasis().getEulerYPR(yaw,pitch,roll);
                measurement(3)= roll;
                measurement(4)= pitch;
                measurement(5)= yaw;


                state_pose_.getBasis().getEulerYPR(yaw,pitch,roll);
                double z = state_pose_.getOrigin().getZ();

                kalman_filter_.correctionStep(measurement,world_to_cam_transform_.inverse(),drone_in_marker_coord_.inverse(),roll_, pitch_, distZ_);

                btQuaternion newRotation;
                newRotation.setEulerZYX(kalman_filter_.state_(2), pitch_, roll_);
                state_pose_.setRotation(newRotation);
                btVector3 newOrigin(kalman_filter_.state_(0),kalman_filter_.state_(1),distZ_);
                state_pose_.setOrigin(newOrigin);
            }


        }
    }
}



void Ardrone::navCB(const multi_drone_ekf::NavdataConstPtr& nav_msg) {
    double dt;

    ROS_INFO_STREAM(
                "------------------------------------------ \n"
                << "Nav_msg vx: " << nav_msg->vx
                << "\nNav_msg vy: " << nav_msg->vy
                << "\nNav_msg vz: " << nav_msg->vz
                << "\nNav_msg yaw: " << nav_msg->rotZ
                << "\nNav_msg height: " << nav_msg->altd);



    if (prevTime_ == 0) {
        prevTime_ = nav_msg->header.stamp.toSec();
        dt = (double) 1 / 14;
        last_yaw_ = (nav_msg->rotZ) / (180.0 / M_PI);
    } else {
        dt = (nav_msg->header.stamp.toSec() - prevTime_);
        prevTime_ = nav_msg->header.stamp.toSec();
    }

    //Calculate changes of translation (incremental translation values)
    double distX = nav_msg->vx * dt / 1000.0;
    double distY = nav_msg->vy * dt / 1000.0;
    distZ_ = (double) (nav_msg->altd) / 1000.0; //altd value in millimeter

    //Get absolute rotation values
    yaw_ = ((nav_msg->rotZ) / (180.0 / M_PI));
    pitch_ = ((nav_msg->rotY) / (180.0 / M_PI));
    roll_ = ((nav_msg->rotX) / (180.0 / M_PI));



    Eigen::Vector3f odometry;

    odometry(0) = distX; // local position update
    odometry(1) = distY; // local position update
    odometry(2) = (yaw_ - last_yaw_) /*/ 180 * M_PI*/; // treat absolute value as incremental update

    //        std::cout<<"odometry: "<<odometry<<std::endl;

    last_yaw_ = yaw_;



    if(initialized_)
    {
        if(!kalman_filter_.initialized_)
        {
            kalman_filter_.init(world_to_drone_pose_);
        }


        kalman_filter_.predictionStep(odometry);


        btQuaternion newRotation;
        newRotation.setEulerZYX(kalman_filter_.state_(2), pitch_, roll_);
        state_pose_.setRotation(newRotation);
        btVector3 newOrigin(kalman_filter_.state_(0),kalman_filter_.state_(1),distZ_);
        state_pose_.setOrigin(newOrigin);





    }


}





Ardrone::Ardrone(uint marker_nr) {


    btVector3 translation(0,0,-0.15);
    btQuaternion rotation;
    rotation.setEulerZYX(-M_PI, 0,0);

    drone_in_marker_coord_.setOrigin(translation);
    drone_in_marker_coord_.setRotation(rotation);

    boost::function<void (const multi_drone_ekf::TagsConstPtr&)> tag_callback( boost::bind(&Ardrone::tagCB, this, _1, marker_nr) );
    sub_tags_ = nh_.subscribe("/tags", 100,  tag_callback);
    prevTime_ = 0;
    tag_seen_first_time_ = false;
    navCB_done_ = false;
    last_yaw_ = 0;
    initialized_ = false;
    distZ_=0;
    yaw_ = 0;
    pitch_ = 0;
    roll_ = 0;



    sub_nav_ = nh_.subscribe("/ardrone/navdata", 100,
                             &Ardrone::navCB, this);
}




