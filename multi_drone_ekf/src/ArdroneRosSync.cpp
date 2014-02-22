#include "multi_drone_ekf/ArdroneRosSync.h"
#include "multi_drone_ekf/MultiAgent3dNavigation.h"


ArdroneRosSync::ArdroneRosSync(ros::NodeHandle &nh, MultiAgent3dNavigation *navigation) :
navigation(navigation)
{
  assert(navigation != NULL);
  cycleDt = navigation->getCycleDt();
    //create agent map
    Agent drone_observer;
    drone_observer.markerId = 1;
    drone_observer.ardroneId = 1;
    agents.insert(std::make_pair(drone_observer.ardroneId, drone_observer));


    std::map<int, Agent>::iterator iter;
    for (iter = agents.begin(); iter != agents.end(); ++iter)
    {
        boost::function<void (const multi_drone_ekf::TagsConstPtr&)> tag_callback( boost::bind(&ArdroneRosSync::tagCB, this, _1, iter->second.ardroneId) );
        std::stringstream ss;
        ss << iter->second.ardroneId;
        std::string tag_topic = "/" + ss.str() + "/tags";
        sub_tags.push_back(nh.subscribe(tag_topic, 100, tag_callback));

        boost::function<void (const multi_drone_ekf::NavdataConstPtr&)> nav_callback( boost::bind(&ArdroneRosSync::navCB, this, _1, iter->second.ardroneId) );
        std::string nav_topic = "/" + ss.str() + "/navdata";
        sub_navs.push_back(nh.subscribe(nav_topic, 100, nav_callback));
    }
}



void ArdroneRosSync::tagCB(const multi_drone_ekf::TagsConstPtr& tag_msg, uint ardroneId)
{
    int tag_cnt = tag_msg->tag_count;

    if (tag_cnt == 0)
        return;

    for (int i = 0; i < tag_cnt; ++i) {
        if (agents[ardroneId].markerId == tag_msg->tags[i].id)
            ROS_DEBUG("Found tag  %i (cf: %.3f)", tag_msg->tags[i].id, tag_msg->tags[i].cf);
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

        tf::Transform tag_pose;
        tag_pose.setOrigin(translation);
        tag_pose.setRotation(rotation);
        tag_pose = tag_pose*pose_around_y;

        agents[ardroneId].measurements.push_back(std::pair<int, tf::Transform>(tag.id, tag_pose));
    }

}


void ArdroneRosSync::navCB(const multi_drone_ekf::NavdataConstPtr& nav_msg, uint ardroneId)
{

    ROS_DEBUG_STREAM(
                "------------------------------------------ \n"
                << "Nav_msg vx: " << nav_msg->vx
                << "\nNav_msg vy: " << nav_msg->vy
                << "\nNav_msg vz: " << nav_msg->vz
                << "\nNav_msg yaw: " << nav_msg->rotZ
                << "\nNav_msg height: " << nav_msg->altd);



    double x = nav_msg->vx / 1000;
    double y = nav_msg->vy / 1000;
    double z = nav_msg->altd / 1000;
    double roll = nav_msg->rotX / (180.0 / M_PI);
    double pitch = nav_msg->rotY / (180.0 / M_PI);
    double yaw = nav_msg->rotZ / (180.0 / M_PI);


    btVector3 odom_trans(x, y, z);
    btQuaternion odom_rot;
    odom_rot.setEulerZYX(yaw, pitch, roll);

    tf::Transform odometry;
    odometry.setOrigin(odom_trans);
    odometry.setRotation(odom_rot);

    agents[ardroneId].odometry = odometry;
}


void ArdroneRosSync::checkCycle() {
  // check if cycleDt passed

  // compute incremental odometry, set last_odometry, set lastCycle, check for first cycle

  // replace marker ids by sensed object ids (also include target)

  // call navigation function

  // publish state estimate

  // publish controls (not in first test)
}
