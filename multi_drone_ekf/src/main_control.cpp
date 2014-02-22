#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <ranav/tparam.h>
#include <tf/transform_broadcaster.h>
#include "multi_drone_ekf/MultiAgent3dNavigation.h"
#include "multi_drone_ekf/Camera.h"
#include "multi_drone_ekf/ArdroneRosSync.h"

using namespace ranav;

int main(int argc, char **argv) {
  std::string paramfile = "multi.ini";
  char c;
  while ((c = getopt(argc, argv, "p:h")) != EOF) {
    switch (c) {
      case 'p':
        paramfile = optarg;
        break;
      case 'h':
      default:
        std::cerr << "Usage: " << argv[0] << " [options]\n";
        std::cerr << "\nOptions:\n";
        std::cerr << "-p <file>:  use the given parameter file\n";
        std::cerr << "-h:         this help\n";
        return 1;
    }
  }

  TParam p;
  p.loadTree(paramfile);

  ros::init(argc, argv, "multi_drone_control");


  Camera camera(12);
  ros::NodeHandle nh_;
  ros::Rate r(60);
  while (nh_.ok() && !camera.pose_set_) {
      ros::spinOnce();
  }

  tf::Quaternion q;
  q.setRPY(0, -M_PI/4, 0);
  tf::Transform drone_to_front_cam(q, tf::Vector3(0.09, 0, 0));
  q.setRPY(0, 0, 0);
  tf::Transform drone_to_marker(q, tf::Vector3(0, 0, 0.15));

  MultiAgent3dNavigation navigation(camera.tag_pose_.inverse(), drone_to_marker, drone_to_front_cam, p);

  ArdroneRosSync rosSync(nh_, &navigation);

  tf::TransformBroadcaster br;


  while (nh_.ok()) {
      ros::spinOnce();
      br.sendTransform(
                  tf::StampedTransform(camera.tag_pose_.inverse(), ros::Time::now(), "/world",
                                       "/kinect"));

  }

  return 0;
}
