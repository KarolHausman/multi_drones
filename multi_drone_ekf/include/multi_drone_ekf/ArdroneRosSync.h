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

class MultiAgent3dNavigation;

class ArdroneRosSync {
public:
  //! Creates the list of agents. Subscribes to the topics of all agents. Sets navigation function.
  ArdroneRosSync(ros::NodeHandle &nh);
  ~ArdroneRosSync();

  //! Handles all incoming marker observations and stores them as measurements of the given ardroneId
  void tagCB(const multi_drone_ekf::TagsConstPtr& tag_msg, int ardroneId);
  //! Handles all incoming odometry data and stores them for the given ardroneId
  void navCB(const multi_drone_ekf::NavdataConstPtr& nav_msg, int ardroneId);

  //! checks whether the collected data is complete to run a cycle of the navigation algorithm. Send the control command(s)
  void checkCycle();

protected:
  struct Agent {
    bool operator<(const Agent &other) const { return ardroneId < other.ardroneId; }
    int markerId;
    int ardroneId;
    // store odometry data
    // collect measurements
  };
  std::set<Agent> agents;
  ros::Time lastCycle; //!< the time when the last cycle was executed
  double cycleDt; //!< duration of one cycle

  MultiAgent3dNavigation *navigation;
};

#endif
