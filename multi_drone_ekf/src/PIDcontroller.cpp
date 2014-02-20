/*
 * pid_controller.cpp
 *
 *  Created on: Feb 20, 2014
 *      Author: Karol Hausman
 */

#include "multi_drone_ekf/PIDcontroller.h"

float PIDcontroller::getCommand(float error, ros::Time current) {

	ros::Duration delta_t=current-last_time;
//	float d_part= ((error-last_error)/delta_t.toSec())*c_derivative;
	last_error=error;
	last_time=current;
	//P-controller
    return (c_proportional * error);//+d_part;
}

