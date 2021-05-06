#ifndef MY_SYNCHRONIZER_HPP
#define MY_SYNCHRONIZER_HPP

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

using namespace message_filters;

typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, \
											sensor_msgs::Image, synchronizer::SemanticObjectArray, \
											sensor_msgs::Image> MySyncPolicy;

class my_Synchronizer{
    public:
        my_Synchronizer(ros::NodeHandle& nh);
        ~my_Synchronizer();



        void callback(const sensor_msgs::Image::ConstPtr& f1, \
	          const sensor_msgs::Image::ConstPtr& m1, \
			  const sensor_msgs::Image::ConstPtr& d1, \
			  const synchronizer::SemanticObjectArray::ConstPtr& s1, \
			  const sensor_msgs::Image::ConstPtr& r1);

    private:
        ros::Publisher toVdo_pub;
        
        message_filters::Subscriber<sensor_msgs::Image> f_sub;
        message_filters::Subscriber<sensor_msgs::Image> m_sub;
        message_filters::Subscriber<sensor_msgs::Image> d_sub;
        message_filters::Subscriber<synchronizer::SemanticObjectArray> s_sub;
        message_filters::Subscriber<sensor_msgs::Image> r_sub;
        Synchronizer<MySyncPolicy> sync;

};


#endif












