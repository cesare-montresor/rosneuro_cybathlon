#include <ros/ros.h>
#include "rosneuro_cybathlon/Control.hpp"

int main(int argc, char** argv) {

	
	// ros initialization
	ros::init(argc, argv, "cybathlon_control");

	rosneuro::Control control;

	if(control.Run() == false)
		ROS_ERROR("Control interrupted while running");

	ros::shutdown();
	return 0;
}
