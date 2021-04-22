#include "ros/ros.h"
#include <sstream>
#include <bits/stdc++.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "synchronizer/Sync.h"
#include "synchronizer/SemanticObjectArray.h"
#include <std_msgs/String.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace message_filters;

static const std::string cvWindow1 = "flownet image";
static const std::string cvWindow2 = "maskrcnn image";

void displayImage(std::string windowName, cv::Mat img){
	cv::namedWindow(windowName, CV_WINDOW_NORMAL);
	cv::imshow(windowName, img);
	cv::waitKey(3);
}

void callback(const sensor_msgs::Image::ConstPtr& f1, \
	          const sensor_msgs::Image::ConstPtr& m1, \
			  const sensor_msgs::Image::ConstPtr& d1, \
			  const synchronizer::SemanticObjectArray::ConstPtr& s1, \
			  const sensor_msgs::Image::ConstPtr& r1) {

	ROS_INFO("This is the combined node output");
	ROS_INFO("f-m-d-s-r timestamps: %.2f/%.2f/%.2f/%.2f/%.2f", f1->header.stamp.toSec(), \
													m1->header.stamp.toSec(), \
													d1->header.stamp.toSec(), \
													s1->header.stamp.toSec(), \
													r1->header.stamp.toSec());

//  to test and see the actual images from the synced topic. 
// 	Note: need to change the data in flownet and maskrcnn topics to visualizable ones**

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

	
	

}



int main(int argc, char** argv) {

	ros::init(argc, argv, "synchronizer_node");
	ros::NodeHandle nh;
	
	message_filters::Subscriber<sensor_msgs::Image> f_sub(nh, "/flownet/flownet_raw", 10);
	message_filters::Subscriber<sensor_msgs::Image> m_sub(nh, "/maskrcnn/maskrcnn_raw", 10);
	message_filters::Subscriber<sensor_msgs::Image> d_sub(nh, "/zed2/zed_node/depth/depth_registered", 10);
	message_filters::Subscriber<synchronizer::SemanticObjectArray> s_sub(nh, "/maskrcnn/maskrcnn_sObj", 10);
	message_filters::Subscriber<sensor_msgs::Image> r_sub(nh, "/zed2/zed_node/left/image_rect_color", 10);

	typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, \
											sensor_msgs::Image, synchronizer::SemanticObjectArray, \
											sensor_msgs::Image> MySyncPolicy;

	

	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), f_sub, m_sub, d_sub, s_sub, r_sub);
	sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4, _5));

	
	ros::spin();

	return 0;

}


////////////////////////////////////////////v2 not working
// class Synchronizer
// {
// 	private:
//         ros::NodeHandle nh;
//         // std::string _cvWindow_name = "image 1";

// 	public:
//         // init
//         Synchronizer(){ //gives the nodehandle to it (as required by the package)
            
// 			message_filters::Subscriber<synchronizer::Sync> f_sub(nh, "/flownet_raw", 5);
// 			message_filters::Subscriber<synchronizer::Sync> s_sub(nh, "/maskrcnn_raw", 5);

// 			typedef sync_policies::ApproximateTime<synchronizer::Sync, synchronizer::Sync> MySyncPolicy;

// 			this<MySyncPolicy> sync(MySyncPolicy(10), f_sub, s_sub);
// 			sync.registerCallback(boost::bind(&Synchronizer::callback,this, _1, _2));
//             // cv::namedWindow(_cvWindow_name, CV_WINDOW_NORMAL);
//         }
//         ~Synchronizer(){
//             // cv::destroyWindow(_cvWindow_name);
//         }
// 		void callback(const synchronizer::Sync::ConstPtr& f1, 
// 	          const synchronizer::Sync::ConstPtr& s1) {

// 				std_msgs::String out_String;

// 				// out_String.data = f1->st + s1->st;
// 				out_String.data = "This is the combined node output";
// 				ROS_INFO_STREAM(out_String);
	
// 		}	
// };

// int main(int argc, char** argv) {

// 	ros::init(argc, argv, "synchronizer_node");
// 	Synchronizer abc;

// 	ros::spin();
// 	return 0;

// }