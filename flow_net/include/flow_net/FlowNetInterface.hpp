#ifndef _FLOWNET_ROS_INTERFACE
#define _FLOWNET_ROS_INTERFACE

// ROS Headers
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cv_bridge/cv_bridge.h>


#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

#include <python_service_starter/ServiceStarterInterface.hpp>

// Standard Library Headers
#include <string>
#include <thread>
#include <vector>
#include <map>

namespace flow_net {

    class FlowNetInterface : public ServiceStarterInterface {

        public:
            FlowNetInterface(ros::NodeHandle& n);
            ~FlowNetInterface() {};

            
            /**
             * @brief Calls the Flow Net service to generate dense optical flow using a pair of images.
             * 
             * @param current_image Input RGB uint8 image of the current frame
             * @param previous_image Input RGB uint8 image of the previous frame
             * @param dst The resulting optical flow. This will be a N X M X 2 (float 32)
             * @param viz The resulting optical flow visualisaed using magma transforms (resulting images
             * is a RGB uint8 (N X M X 3))
             * @return true 
             * @return false 
             */
            bool analyse(const cv::Mat& current_image, const cv::Mat& previous_image, cv::Mat& dst, cv::Mat& viz);

            /**
             * @brief begins the python program found in the scripts file.
             * 
             * @param wait_for_services if true the function should use the wait for services function to wait for the 
             * appropiate services to be ready
             * @return true 
             * @return false 
             */
            bool start_service(bool wait_for_services = true) override;

            /**
             * @brief Waits for the required service to be up: in this case 
             * 'flownet/analyse_image'
             * 
             * @param timeout 
             * @return true 
             * @return false 
             */
            static bool wait_for_services(ros::Duration timeout = ros::Duration(-1));    

    };

}; //namespace flow_net



#endif
