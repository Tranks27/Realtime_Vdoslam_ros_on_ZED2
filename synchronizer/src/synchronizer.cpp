#include "ros/ros.h"
#include <sstream>
#include <bits/stdc++.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "synchronizer/Sync.h"
#include <std_msgs/String.h>

using namespace message_filters;

void callback(const synchronizer::Sync::ConstPtr& f1, 
	          const synchronizer::Sync::ConstPtr& s1) {

	std_msgs::String out_String;

	// out_String.data = f1->st + s1->st;
    out_String.data = "This is the combined node output";
	ROS_INFO_STREAM(out_String);
}


int main(int argc, char** argv) {

	ros::init(argc, argv, "synchronizer_node");
	ros::NodeHandle nh;
	
	message_filters::Subscriber<synchronizer::Sync> f_sub(nh, "/flownet_raw", 5);
	message_filters::Subscriber<synchronizer::Sync> s_sub(nh, "/maskrcnn_raw", 5);

	typedef sync_policies::ApproximateTime<synchronizer::Sync, synchronizer::Sync> MySyncPolicy;

	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), f_sub, s_sub);
	sync.registerCallback(boost::bind(&callback, _1, _2));
	ROS_INFO_STREAM("checking ...");
	
	ros::spin();

	return 0;

}