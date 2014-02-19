/*
 *
 *  Created on: Feb 6, 2014
 *      Author: Karol Hausman
 */


#include "multi_drone_ekf/Ardrone.h"
#include "multi_drone_ekf/Camera.h"

struct Marker {

    std::string tags_topic_;
    tf::Transform tag_pose_;
    ros::NodeHandle nh_;
    ros::Subscriber sub_tags_;
    bool updated_;


    Marker(int marker_nr)
    {
        tags_topic_ = "/tags";
        updated_ = false;
        boost::function<void (const multi_drone_ekf::TagsConstPtr&)> tag_callback( boost::bind(&Marker::tagCB, this, _1, marker_nr) );
        sub_tags_ = nh_.subscribe(tags_topic_, 100,  tag_callback);
    }


    void tagCB(const multi_drone_ekf::TagsConstPtr& tag_msg, uint marker) {

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

            }
        }
        updated_ =true;
    }



};




int main(int argc, char **argv) {
    ros::init(argc, argv, "multi_drone_ekf");

    Ardrone drone_observer(1);
    Camera camera(12);


    tf::TransformBroadcaster br;

    bool init = true;

    ros::Rate r(30);
    while (drone_observer.nh_.ok()) {
        ros::spinOnce();




        if (camera.pose_set_)
        {


            drone_observer.world_to_cam_transform_ = camera.tag_pose_.inverse();

            br.sendTransform(
                        tf::StampedTransform(drone_observer.world_to_cam_transform_, ros::Time::now(), "/zeta_marker",
                                             "/camera"));

            br.sendTransform(
                        tf::StampedTransform(drone_observer.world_to_cam_transform_*drone_observer.tag_pose_, ros::Time::now(), "/zeta_marker",
                                             "/beta_marker"));


            if(init)
            {

                if(drone_observer.tag_seen_first_time_)
                {

                    drone_observer.initialized_ = true;

                    init = false;

                    drone_observer.world_to_drone_pose_ = drone_observer.world_to_cam_transform_*drone_observer.tag_pose_*drone_observer.drone_in_marker_coord_;

                }
            }

            br.sendTransform(
                        tf::StampedTransform(drone_observer.state_pose_, ros::Time::now()/*nav_msg->header.stamp*/,
                                             "/zeta_marker", "/ardrone"));

        }

        r.sleep();
    }

    return 0;
}
