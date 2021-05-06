#include "ros/ros.h"
#include <sstream>
#include <bits/stdc++.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "synchronizer/Sync.h"
#include "synchronizer/SemanticObjectArray.h"
#include <my_realtime_vdo_slam/VdoInput.h>
#include <std_msgs/String.h>

#include "my_Synchronizer.hpp"

using namespace message_filters;

my_Synchronizer::my_Synchronizer(ros::NodeHandle& nh):sync(MySyncPolicy(10), f_sub, m_sub, d_sub, s_sub, r_sub){
	toVdo_pub = nh.advertise<my_realtime_vdo_slam::VdoInput>("/vdoslam/input/all",100);

	f_sub.subscribe(nh, "/flownet/flownet_raw", 10);
	m_sub.subscribe(nh, "/maskrcnn/maskrcnn_raw", 10);
	d_sub.subscribe(nh, "/zed2/zed_node/depth/depth_registered", 10);
	s_sub.subscribe(nh, "/maskrcnn/maskrcnn_sObj", 10);
	r_sub.subscribe(nh, "/zed2/zed_node/left/image_rect_color", 10);

	sync.registerCallback(boost::bind(&my_Synchronizer::callback, this,_1, _2, _3, _4, _5));
}

my_Synchronizer::~my_Synchronizer(){
	
}

void my_Synchronizer::callback(const sensor_msgs::Image::ConstPtr& f1, \
						const sensor_msgs::Image::ConstPtr& m1, \
						const sensor_msgs::Image::ConstPtr& d1, \
						const synchronizer::SemanticObjectArray::ConstPtr& s1, \
						const sensor_msgs::Image::ConstPtr& r1) {



				ROS_INFO("This is the combined node output");
				// ROS_INFO("f-m-d-s-r timestamps: %.2f/%.2f/%.2f/%.2f/%.2f", f1->header.stamp.toSec(), \
				// 												m1->header.stamp.toSec(), \
				// 												d1->header.stamp.toSec(), \
				// 												s1->header.stamp.toSec(), \
				// 												r1->header.stamp.toSec());

				my_realtime_vdo_slam::VdoInput msg;
				msg.header.stamp = r1->header.stamp;
				msg.rgb = *r1;
				msg.flow = *f1;
				msg.mask = *m1;
				msg.depth = *d1;
				msg.semantic_objects = s1->semantic_objects;
				toVdo_pub.publish(msg);

}



/**********************************************************/
//  to test and see the actual images from the synced topic. 
// 	Note: need to change the data in flownet and maskrcnn topics to visualizable ones**

// static const std::string cvWindow1 = "flownet image";
// static const std::string cvWindow2 = "maskrcnn image";
// void displayImage(std::string windowName, cv::Mat img){
// 	cv::namedWindow(windowName, CV_WINDOW_NORMAL);
// 	cv::imshow(windowName, img);
// 	cv::waitKey(3);
// }

	// cv_bridge::CvImagePtr cv_ptr1;
	// try{
	// 	cv_ptr1 = cv_bridge::toCvCopy(f1, sensor_msgs::image_encodings::BGR8);
	// }catch (cv_bridge::Exception& e){
	// 	ROS_ERROR("error %s", e.what());
	// 	return;
	// }
	// displayImage(cvWindow1, cv_ptr1->image);

	// cv_bridge::CvImagePtr cv_ptr2;
	// try{
	// 	cv_ptr2 = cv_bridge::toCvCopy(m1, sensor_msgs::image_encodings::BGR8);
	// }catch (cv_bridge::Exception& e){
	// 	ROS_ERROR("error %s", e.what());
	// 	return;
	// }

	
	// displayImage(cvWindow2, cv_ptr2->image);