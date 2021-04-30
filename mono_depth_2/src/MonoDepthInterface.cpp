#include "mono_depth_2/MonoDepthInterface.hpp"

#include <mono_depth_2/MonoDepth.h>
#include <cv_bridge/cv_bridge.h>
#include <python_service_starter/StartMonoDepth.h>

using namespace mono_depth_2;

MonoDepthInterface::MonoDepthInterface(ros::NodeHandle& n) :
        ServiceStarterInterface(n)
    {
    
    service_started = false;
    start_client = nh.serviceClient<python_service_starter::StartMonoDepth>("start_mono_depth");

    }

bool MonoDepthInterface::start_service(bool wait_for_services) {

    if (ros::service::exists("monodepth2/analyse_image", true)) {
        return true;
    }

    python_service_starter::StartMonoDepth srv;
    srv.request.start = true;

    service_started = start_client.call(srv);
    ROS_INFO_STREAM("Start Mono Depth service returned " << service_started);
    client  = nh.serviceClient<mono_depth_2::MonoDepth>("monodepth2/analyse_image");

    if (wait_for_services) {
        return MonoDepthInterface::wait_for_services();
    }

    return service_started;
    
}

bool MonoDepthInterface::wait_for_services(ros::Duration timeout) {
    return ros::service::waitForService("monodepth2/analyse_image", timeout);
}


bool MonoDepthInterface::analyse(const cv::Mat& current_image, cv::Mat& dst) {


    if (!service_started) {
        return false;
    }


    sensor_msgs::ImagePtr current_image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", current_image).toImageMsg();

    mono_depth_2::MonoDepth srv;
    srv.request.current_image = *current_image_msg;

    if(client.call(srv)) {

        if (srv.response.success) {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(srv.response.output_image, sensor_msgs::image_encodings::MONO16);
            cv::Mat image = cv_ptr->image;

            //do i need to copy here?
            dst = image;
            return true;
        }
        else {
            ROS_ERROR_STREAM("Mono Depth service returned failed success");
            return false;
        }
        
    }
    else {
        ROS_ERROR_STREAM("Failed to call Mono Depth service");
        return false;
    }


}