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
                Eigen::VectorXd measurement(6);
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




                tf::Transform drone_to_marker_transform = drone_in_marker_coord_.inverse();


                tf::Transform drone_to_marker_flat = drone_to_marker_transform;
                double d_yaw = 0;
                double d_pitch = 0;
                double d_roll = 0;
                drone_to_marker_flat.getBasis().getEulerYPR(d_yaw, d_pitch, d_roll);
                btVector3 d2m_origin_flat(drone_to_marker_flat.getOrigin().getX(),drone_to_marker_flat.getOrigin().getY(),0);
                drone_to_marker_flat.setOrigin(d2m_origin_flat);
                btQuaternion d2m_quaternion_flat;
                d2m_quaternion_flat.setEulerZYX(d_yaw,0,0);
                drone_to_marker_flat.setRotation(d2m_quaternion_flat);




                tf::Transform cam_to_world_transform = world_to_cam_transform_.inverse();

                tf::Transform cam_to_world_flat;
                double c_yaw = 0;
                double c_pitch = 0;
                double c_roll = 0;
                cam_to_world_transform.inverse().getBasis().getEulerYPR(c_yaw,c_pitch,c_roll);
                btVector3 c2w_origin_flat(cam_to_world_transform.inverse().getOrigin().getX(),cam_to_world_transform.inverse().getOrigin().getY(),0);
                cam_to_world_flat.setOrigin(c2w_origin_flat);
                btQuaternion c2w_quaternion_flat;
                c2w_quaternion_flat.setEulerZYX(c_yaw,0,0);
                cam_to_world_flat.setRotation(c2w_quaternion_flat);

                cam_to_world_flat = cam_to_world_flat.inverse();




                sensorModel = new ranav::Marker3dSensorModel(cam_to_world_flat, drone_to_marker_flat);
                sensorModel->setNoiseCov(world_to_cam_transform_,measurement);

                Eigen::VectorXd measurement_3dog = Eigen::Vector3d::Zero();
                measurement_3dog = sensorModel->downProjectMeasurement(measurement, world_to_cam_transform_);

                kalman_filter_->correctionStep(measurement_3dog, *sensorModel);

                btQuaternion newRotation;
                newRotation.setEulerZYX(kalman_filter_->state_(2), pitch_, roll_);
                state_pose_.setRotation(newRotation);
                btVector3 newOrigin(kalman_filter_->state_(0),kalman_filter_->state_(1),distZ_);
                state_pose_.setOrigin(newOrigin);
            }


        }
    }
}



void Ardrone::navCB(const multi_drone_ekf::NavdataConstPtr& nav_msg) {

    ROS_INFO_STREAM(
                "------------------------------------------ \n"
                << "Nav_msg vx: " << nav_msg->vx
                << "\nNav_msg vy: " << nav_msg->vy
                << "\nNav_msg vz: " << nav_msg->vz
                << "\nNav_msg yaw: " << nav_msg->rotZ
                << "\nNav_msg height: " << nav_msg->altd);



    double dt;
    if (prevTime_ == 0) {
        prevTime_ = nav_msg->header.stamp.toSec();
        dt = (double) 1 / 14;
        last_roll_ = (nav_msg->rotX) / (180.0 / M_PI);
        last_pitch_ = (nav_msg->rotY) / (180.0 / M_PI);
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



    Eigen::VectorXd odometry;

    odometry = Eigen::VectorXd::Zero(6);

    odometry(0) = distX; // local position update
    odometry(1) = distY; // local position update
    odometry(2) = distZ_ /*/ 180 * M_PI*/; // treat absolute value as incremental update
    odometry(3) = (roll_ - last_roll_)/*/ 180 * M_PI*/; // treat absolute value as incremental update
    odometry(4) = (pitch_ - last_pitch_)  /*/ 180 * M_PI*/; // treat absolute value as incremental update
    odometry(5) = (yaw_ - last_yaw_) /*/ 180 * M_PI*/; // treat absolute value as incremental update


    last_yaw_ = yaw_;
    last_roll_ = roll_;
    last_pitch_ = pitch_;




    if(initialized_)
    {
        if(!kalman_filter_->initialized_)
        {
            Eigen::Vector3d mean_init(3);

            mean_init(0) = world_to_drone_pose_.getOrigin().getX();
            mean_init(1) = world_to_drone_pose_.getOrigin().getY();
            double yaw = 0;
            double pitch = 0;
            double roll = 0;
            world_to_drone_pose_.getBasis().getEulerYPR(yaw, pitch, roll);
            mean_init(2) = yaw;

            kalman_filter_->init(mean_init);
        }


        kalman_filter_->predictionStep(motionModel.downProjectControl(odometry));


        btQuaternion newRotation;
        newRotation.setEulerZYX(kalman_filter_->state_(2), pitch_, roll_);
        state_pose_.setRotation(newRotation);
        btVector3 newOrigin(kalman_filter_->state_(0),kalman_filter_->state_(1),distZ_);
        state_pose_.setOrigin(newOrigin);

    }


}





Ardrone::Ardrone(const uint& marker_nr) {

    kalman_filter_ = new ExtendedKalmanFilter(&motionModel);
    btVector3 translation(0,0,-0.15);
    btQuaternion rotation;
    rotation.setEulerZYX(0, 0,0);

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




