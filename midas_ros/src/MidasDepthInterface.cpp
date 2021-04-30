#include "midas_ros/MidasDepthInterface.hpp"

#include <midas_ros/MidasDepth.h>
#include <cv_bridge/cv_bridge.h>
#include <python_service_starter/StartMidasDepth.h>

using namespace midas_ros;

MidasDepthInterface::MidasDepthInterface(ros::NodeHandle& n) :
        ServiceStarterInterface(n)
    {
    
    start_client = nh.serviceClient<python_service_starter::StartMidasDepth>("start_midas_depth");
    ROS_INFO_STREAM("Made start midas depth client");

    }

bool MidasDepthInterface::start_service(bool wait_for_services) {

    if (ros::service::exists("midasdepth/analyse_image", true)) {
        service_started = true;
        return true;
    }

    python_service_starter::StartMidasDepth srv;
    srv.request.start = true;

    service_started = start_client.call(srv);
    ROS_INFO_STREAM("Start Midas depth service returned " << service_started);
    client = nh.serviceClient<midas_ros::MidasDepth>("midasdepth/analyse_image");

    if (wait_for_services) {
        return MidasDepthInterface::wait_for_services();
    }

    return service_started;
    
}

bool MidasDepthInterface::wait_for_services(ros::Duration timeout) {
    return ros::service::waitForService("midasdepth/analyse_image", timeout);
}


bool MidasDepthInterface::analyse(const cv::Mat& current_image, cv::Mat& dst) {

    if (!service_started) {
        return false;
    }

    sensor_msgs::ImagePtr current_image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", current_image).toImageMsg();

    midas_ros::MidasDepth srv;
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
            ROS_ERROR_STREAM("Midas Depth service returned failed success");
            return false;
        }
        
    }
    else {
        ROS_ERROR_STREAM("Failed to call Midas Depth service");
        return false;
    }


}