#ifndef ARDRONEROSSYNC_H_
#define ARDRONEROSSYNC_H_

#include <vector>
#include <set>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include "multi_drone_ekf/Navdata.h"
#include "multi_drone_ekf/Tag.h"
#include "multi_drone_ekf/Tags.h"
#include <sensor_msgs/Joy.h>

class MultiAgent3dNavigation;

class ArdroneRosSync {
public:
  //! Creates the list of agents. Subscribes to the topics of all agents. Sets navigation function.
  ArdroneRosSync(ros::NodeHandle &nh, MultiAgent3dNavigation *navigation);
  ~ArdroneRosSync();

  //! Handles all incoming marker observations and stores them as measurements of the given ardroneId
  void tagCB(const multi_drone_ekf::TagsConstPtr& tag_msg, int ardroneId);
  //! Handles all incoming odometry data and stores them for the given ardroneId
  void navCB(const multi_drone_ekf::NavdataConstPtr& nav_msg, int ardroneId);
  //! handles joystick buttons
  void joystickCB(const sensor_msgs::JoyConstPtr& joy_msg);

  //! checks whether the collected data is complete to run a cycle of the navigation algorithm. Send the control command(s)
  void checkCycle();

protected:
  struct Agent {
    Agent() : gotOdo(false) {}
    bool operator<(const Agent &other) const { return ardroneId < other.ardroneId; }
    int markerId;
    int ardroneId; // for topic
    tf::Transform last_odometry;
    tf::Transform odometry;
    std::vector<std::pair<int, tf::Transform> > measurements; //!< pair: id of sensed marker, measurement transform
    bool gotOdo;
    ros::Publisher pub_control;
  };

  std::map<int, Agent> agents;
  int globalId; // for camera topic
  std::vector<std::pair<int, tf::Transform> > globalMeasurements; //!< pair: id of sensed marker, measurement transform
  int targetMarkerId;
  ros::Time lastCycle; //!< the time when the last cycle was executed
  double cycleDt; //!< duration of one cycle
  std::vector<ros::Subscriber> sub_tags; //!< tags subscriber
  std::vector<ros::Subscriber> sub_navs; //!< navdata subscriber
  tf::TransformBroadcaster transform_broadcaster;
  tf::Transform pose_around_y; //!< transform for the marker detection
  ros::Subscriber sub_joystick;
  bool publishCommands;



  MultiAgent3dNavigation *navigation;
};

#endif
