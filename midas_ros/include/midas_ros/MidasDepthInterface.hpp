#ifndef _MIDAS_ROS_INTERFACE
#define _MIDAS_ROS_INTERFACE

// ROS Headers
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

#include <python_service_starter/ServiceStarterInterface.hpp>

// Standard Library Headers
#include <string>
#include <thread>
#include <vector>
#include <map>

namespace midas_ros {

    class MidasDepthInterface : public ServiceStarterInterface {

        public:
            MidasDepthInterface(ros::NodeHandle& n);
            ~MidasDepthInterface() {};

            /**
             * @brief Calls the Midas depth service in order to inference an RGB image
             * 
             * The input image is a cv::Mat RGB uint8 type image. The dst will be greyscale uint16
             * image.
             * 
             * @param current_image RGB image to analyse
             * @param dst The resulting depth inference. 
             * @return true 
             * @return false 
             */
            bool analyse(const cv::Mat& current_image, cv::Mat& dst);

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
             * 'midasdepth/analyse_image
             * 
             * @param timeout 
             * @return true 
             * @return false 
             */
            static bool wait_for_services(ros::Duration timeout = ros::Duration(-1));    

    };

    typedef std::shared_ptr<MidasDepthInterface> MidasDepthInterfacePtr;

}; //namespace midas_ros



#endif
