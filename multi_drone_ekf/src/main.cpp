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



            tf::Transform world_to_cam = drone_observer.world_to_cam_transform_;

            double c_yaw = 0;
            double c_pitch = 0;
            double c_roll = 0;

            world_to_cam.getBasis().getEulerYPR(c_yaw,c_pitch,c_roll);


            double c_x =0;double c_y=0; //double c_z=0;

            c_x = world_to_cam.getOrigin().getX();
            c_y = world_to_cam.getOrigin().getY();
        //    c_z = world_to_cam.getOrigin().getZ();


            tf::Transform cam_to_marker=drone_observer.tag_pose_;
//            tf::Vector3 origin(measurement(0), measurement(1), measurement(2));
//            cam_to_marker.setOrigin(origin);
//            tf::Quaternion rotation;
//            rotation.setRPY(measurement(3),measurement(4), measurement(5));
//            cam_to_marker.setRotation(rotation);

            tf::Transform world_to_marker = world_to_cam * cam_to_marker;

            double m_yaw = 0;
            double m_pitch = 0;
            double m_roll = 0;

            world_to_marker.getBasis().getEulerYPR(m_yaw,m_pitch,m_roll);

            double m_x =0;double m_y=0; double m_z=0;

            m_x = world_to_marker.getOrigin().getX();
            m_y = world_to_marker.getOrigin().getY();
            m_z = world_to_marker.getOrigin().getZ();

//            std::cout<<"world to marker MAIN: \n"<<"m_x= "<<m_x<<std::endl;
//            std::cout<<"m_y= "<<m_y<<std::endl;
//            std::cout<<"m_z= "<<m_z<<std::endl;
//            std::cout<<"m_roll= "<<m_roll<<std::endl;
//            std::cout<<"m_pitch= "<<m_pitch<<std::endl;
//            std::cout<<"m_yaw= "<<m_yaw<<std::endl;


            tf::Transform c_3d,z_3d,m_3d;

            tf::Vector3 c_3d_origin(c_x,c_y,0);
            tf::Quaternion c_3d_rotation;
            c_3d_rotation.setRPY(0,0,c_yaw);
            c_3d.setOrigin(c_3d_origin);
            c_3d.setRotation(c_3d_rotation);



            tf::Vector3 m_3d_origin(m_x,m_y,0);
            tf::Quaternion m_3d_rotation;
            m_3d_rotation.setRPY(0,0,m_yaw);
            m_3d.setOrigin(m_3d_origin);
            m_3d.setRotation(m_3d_rotation);


            z_3d = c_3d.inverse()*m_3d;

            m_3d = c_3d*z_3d;

//            br.sendTransform(
//            tf::StampedTransform(m_3d, ros::Time::now()/*nav_msg->header.stamp*/,
//                    "/zeta_marker", "/measurement_3dog"));




            Eigen::Vector3f h;

            tf::Transform state_pose;
            btVector3 state_origin(drone_observer.kalman_filter_.state_(0),drone_observer.kalman_filter_.state_(1),drone_observer.distZ_);
            state_pose.setOrigin(state_origin);
            btQuaternion state_quaternion;
            state_quaternion.setEulerZYX(drone_observer.kalman_filter_.state_(2),drone_observer.pitch_,drone_observer.roll_);
            state_pose.setRotation(state_quaternion);


            tf::Transform H_transform;
            H_transform = drone_observer.world_to_cam_transform_.inverse()*state_pose*drone_observer.drone_in_marker_coord_.inverse();

            double h_yaw = 0;
            double h_pitch = 0;
            double h_roll = 0;

            H_transform.getBasis().getEulerYPR(h_yaw, h_pitch, h_roll);

            h << H_transform.getOrigin().getX(),H_transform.getOrigin().getY(),h_yaw;


            tf::Transform h_3d;
            tf::Vector3 h_3d_origin(H_transform.getOrigin().getX(),H_transform.getOrigin().getY(),0);
            tf::Quaternion h_3d_rotation;
            h_3d_rotation.setRPY(0,0,h_yaw);
            h_3d.setOrigin(h_3d_origin);
            h_3d.setRotation(h_3d_rotation);

//            br.sendTransform(
//            tf::StampedTransform(c_3d*h_3d, ros::Time::now()/*nav_msg->header.stamp*/,
//                    "/zeta_marker", "/H_FUNCTION"));

//            tf::Transform world_to_beta = drone_observer.world_to_cam_transform_*drone_observer.tag_pose_;

//            tf::Transform marker_projection /*= camera.tag_pose_ * world_to_beta*/;











/*
            tf::Transform cam_to_world = camera.tag_pose_;
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

*/




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



            tf::Transform beta_from_H ;//= drone_observer.world_to_cam_transform_.inverse()*state_pose*drone_observer.drone_in_marker_coord_.inverse();

            tf::Transform state_pose_flat;
            btVector3 state_origin_flat(drone_observer.kalman_filter_.state_(0),drone_observer.kalman_filter_.state_(1),0);
            state_pose_flat.setOrigin(state_origin_flat);
            btQuaternion state_quaternion_flat;
            state_quaternion_flat.setEulerZYX(drone_observer.kalman_filter_.state_(2),0,0);
            state_pose_flat.setRotation(state_quaternion_flat);


            tf::Transform drone_to_marker_flat = drone_observer.drone_in_marker_coord_.inverse();
            double d_yaw = 0;
            double d_pitch = 0;
            double d_roll = 0;

            drone_to_marker_flat.getBasis().getEulerYPR(d_yaw, d_pitch, d_roll);
            btVector3 d2m_origin_flat(drone_to_marker_flat.getOrigin().getX(),drone_to_marker_flat.getOrigin().getY(),0);
            drone_to_marker_flat.setOrigin(d2m_origin_flat);
            btQuaternion d2m_quaternion_flat;
            drone_to_marker_flat.getBasis().getEulerYPR(d_yaw,d_pitch,d_roll);
            d2m_quaternion_flat.setEulerZYX(d_yaw,0,0);
            drone_to_marker_flat.setRotation(d2m_quaternion_flat);




            beta_from_H =c_3d.inverse()*state_pose_flat*drone_to_marker_flat;



            tf::Vector3 beta_3d_origin(beta_from_H.getOrigin().getX(),beta_from_H.getOrigin().getY(),0);

            double beta_yaw = 0;
            double beta_pitch = 0;
            double beta_roll = 0;

            beta_from_H.getBasis().getEulerYPR(beta_yaw, beta_pitch, beta_roll);

            tf::Quaternion beta_3d_rotation;
            beta_3d_rotation.setRPY(0,0,beta_yaw);
            beta_from_H.setOrigin(beta_3d_origin);
            beta_from_H.setRotation(beta_3d_rotation);

            Eigen::Vector3f beta_h;
            beta_h << beta_from_H.getOrigin().getX(),beta_from_H.getOrigin().getY(),beta_yaw;

//            std::cout<<"h from MAIN: \n"<<beta_h<<std::endl;

            tf::Transform debug_cam = c_3d.inverse();
            Eigen::Vector3f debug_cam2;
            debug_cam2 << debug_cam.getOrigin().getX(),debug_cam.getOrigin().getY(),tf::getYaw(debug_cam.getRotation());
//            std::cout<< "debug cam2: \n"<<debug_cam2<<std::endl;




            br.sendTransform(
            tf::StampedTransform(drone_observer.world_to_cam_transform_.inverse()*state_pose*drone_observer.drone_in_marker_coord_.inverse(), ros::Time::now()/*nav_msg->header.stamp*/,
                    "/camera", "/beta_from_H"));


//            br.sendTransform(
//            tf::StampedTransform(c_3d*beta_from_H, ros::Time::now()/*nav_msg->header.stamp*/,
//                    "/zeta_marker", "/beta_from_H2"));






        }

//        br.sendTransform(
//                tf::StampedTransform(drone_observer.tag_pose_, ros::Time::now(), "/camera",
//                        "/beta_marker"));



		r.sleep();
	}

	return 0;
}
