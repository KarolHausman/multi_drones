/*
 * pid_controller.h
 *
 *  Created on: Feb 20, 2014
 *      Author: Karol Hausman
 */

#ifndef PIDCONTROLLER_H_
#define PIDCONTROLLER_H_
#include <ros/ros.h>
#include <limits>

struct PIDcontroller {


  float c_proportional;
  float c_integral;
  float c_derivative;

  // calculation of new command
  float getCommand(float error, ros::Time current);


  PIDcontroller(){
   c_proportional = c_integral = c_derivative = error_sum = 0;
   last_error = std::numeric_limits<float>::max();

  }

private:

  // useful for d-part
  ros::Time last_time;
  float last_error;
  
  // useful for i-part
  float error_sum; 


};



#endif /* PDICONTROLLER_H_ */

