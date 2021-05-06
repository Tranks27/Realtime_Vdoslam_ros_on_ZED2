#include "ros/ros.h"
#include "my_Synchronizer.hpp"



int main(int argc, char** argv) {

	ros::init(argc, argv, "synchronizer_node");
	ros::NodeHandle nh;
	my_Synchronizer my_Synchronizer(nh);

    ROS_INFO("I run!");
	
    ros::spin();

	return 0;

}