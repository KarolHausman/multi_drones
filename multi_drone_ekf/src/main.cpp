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

//            if (tag.id != marker) {
//                ROS_INFO("Detected unknown Marker");
//                return;
//            }

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
	ros::init(argc, argv, "ardrone_visualization");

	// To test your correction step:
	//	ExtendedKalmanFilter EKF;
	//	EKF.testFilter();
	//	return 0;

    Ardrone drone_observer(1);
    Camera camera(12);


	tf::TransformBroadcaster br;

    bool init = true;

	ros::Rate r(30);
    while (drone_observer.nh_.ok()) {
		ros::spinOnce();




        if (camera.pose_set_)
        {
//            br.sendTransform(
//                    tf::StampedTransform(camera.tag_pose_, ros::Time::now(),
//                            "/camera","/zeta_marker"));


            drone_observer.world_to_cam_transform_ = camera.tag_pose_.inverse();

            br.sendTransform(
                    tf::StampedTransform(drone_observer.world_to_cam_transform_, ros::Time::now(), "/zeta_marker",
                            "/camera"));

            br.sendTransform(
                    tf::StampedTransform(drone_observer.world_to_cam_transform_*drone_observer.tag_pose_, ros::Time::now(), "/zeta_marker",
                            "/beta_marker"));


//            tf::Transform world_to_beta = drone_observer.world_to_cam_transform_*drone_observer.tag_pose_;

            tf::Transform cam_to_world = camera.tag_pose_;
//            tf::Transform marker_projection /*= camera.tag_pose_ * world_to_beta*/;












            tf::Transform world_to_cam = drone_observer.world_to_cam_transform_;
            tf::Transform cam_to_marker = drone_observer.tag_pose_;
            tf::Transform state_6D = world_to_cam*cam_to_marker;

            double c_yaw = 0;
            double c_pitch = 0;
            double c_roll = 0;

            world_to_cam.getBasis().getEulerYPR(c_yaw,c_pitch,c_roll);

            double c_x =0;double c_y=0; double c_z=0;

            c_x = cam_to_world.getOrigin().getX();
            c_y = cam_to_world.getOrigin().getY();
            c_z = cam_to_world.getOrigin().getZ();






            double p_yaw = 0;
            double p_pitch = 0;
            double p_roll = 0;

            state_6D.getBasis().getEulerYPR(p_yaw,p_pitch,p_roll);

            double p_x =0; double p_y=0; double p_z=0;

            p_x = state_6D.getOrigin().getX();
            p_y = state_6D.getOrigin().getY();
            p_z = state_6D.getOrigin().getZ();










            double z_yaw = 0;
            double z_pitch = 0;
            double z_roll = 0;

            cam_to_marker.getBasis().getEulerYPR(z_yaw,z_pitch,z_roll);

            double z_x =0;double z_y=0; double z_z=0;

            z_x = cam_to_marker.getOrigin().getX();
            z_y = cam_to_marker.getOrigin().getY();
            z_z = cam_to_marker.getOrigin().getZ();





            Eigen::Matrix3d c_3d;
            Eigen::Matrix3d z_3d;
            Eigen::Matrix3d p_3d;

            c_3d << cos(c_yaw),-sin(c_yaw),c_x,
                    sin(c_yaw),cos(c_yaw),c_y,
                    0,0,1;


            p_3d << cos(p_yaw),-sin(p_yaw),p_x,
                    sin(p_yaw),cos(p_yaw),p_y,
                    0,0,1;

            Eigen::Matrix3d p_3d_to_check;

            z_3d = c_3d.inverse()*p_3d;

            p_3d_to_check = c_3d*z_3d;


//            std::cout<<"real: \n"<<p_3d<<std::endl;

//            std::cout<<"to check: \n"<<p_3d_to_check<<std::endl;






            if(init)
            {

                    if(drone_observer.tag_seen_first_time_)
                    {

                        drone_observer.initialized_ = true;

                        init = false;

                        drone_observer.world_to_drone_pose_ = drone_observer.world_to_cam_transform_*drone_observer.tag_pose_*drone_observer.drone_in_marker_coord_;

                    }
            }





//            br.sendTransform(
//            tf::StampedTransform(camera.tag_pose_.inverse()*drone_observer.tag_pose_*drone_observer.drone_in_marker_coord_, ros::Time::now()/*nav_msg->header.stamp*/,
//                    "/zeta_marker", "/ardrone"));


            br.sendTransform(
            tf::StampedTransform(drone_observer.world_to_drone_pose_, ros::Time::now()/*nav_msg->header.stamp*/,
                    "/zeta_marker", "/init"));


            br.sendTransform(
            tf::StampedTransform(drone_observer.state_pose_, ros::Time::now()/*nav_msg->header.stamp*/,
                    "/zeta_marker", "/ardrone"));


            br.sendTransform(
            tf::StampedTransform(drone_observer.world_to_cam_transform_.inverse()*drone_observer.state_pose_*drone_observer.drone_in_marker_coord_.inverse(), ros::Time::now()/*nav_msg->header.stamp*/,
                    "/camera", "/beta_from_H"));






            tf::Transform H_transform = drone_observer.world_to_cam_transform_.inverse()*drone_observer.state_pose_*drone_observer.drone_in_marker_coord_.inverse();

            double h_yaw = 0;
            double h_pitch = 0;
            double h_roll = 0;

            H_transform.getBasis().getEulerYPR(h_yaw, h_pitch, h_roll);

            Eigen::Vector3f h;


            h << H_transform.getOrigin().getX(),H_transform.getOrigin().getY(),h_yaw;
//            std::cout<<"h from main: "<<h<<std::endl;

            Eigen::Vector3f measurement;
            measurement(0) = drone_observer.tag_pose_.getOrigin().getX();
            measurement(1) = drone_observer.tag_pose_.getOrigin().getY();
            double roll = 0;
            double pitch = 0;
            double yaw = 0;
            drone_observer.tag_pose_.getBasis().getEulerYPR(yaw,pitch,roll);
            measurement(2)= yaw;

            Eigen::Vector3f brackets2 = measurement - h;
            //normalize yaw angle
            brackets2(2) = atan2(sin(brackets2(2)), cos(brackets2(2)));

            std::cerr<<"FROM MAIN measurement - h: "<<brackets2<<std::endl;



        }

//        br.sendTransform(
//                tf::StampedTransform(drone_observer.tag_pose_, ros::Time::now(), "/camera",
//                        "/beta_marker"));



		r.sleep();
	}

	return 0;
}
