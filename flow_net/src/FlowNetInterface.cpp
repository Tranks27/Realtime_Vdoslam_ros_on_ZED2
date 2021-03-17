#include "flow_net/FlowNetInterface.hpp"

#include <flow_net/FlowNet.h>
#include <cv_bridge/cv_bridge.h>
#include <python_service_starter/StartFlowNet.h>

using namespace flow_net;

FlowNetInterface::FlowNetInterface(ros::NodeHandle& n) :
        ServiceStarterInterface(n)
    {
      
    start_client = nh.serviceClient<python_service_starter::StartFlowNet>("start_flow_net");

    }

bool FlowNetInterface::start_service(bool wait_for_services) {

     if (ros::service::exists("flownet/analyse_image", true)) {
        return true;
    }


    python_service_starter::StartFlowNet srv;
    srv.request.start = true;

    service_started = start_client.call(srv);
    ROS_INFO_STREAM("Start flow net service returned " << service_started);
    client  = nh.serviceClient<flow_net::FlowNet>("flownet/analyse_image");

    if (wait_for_services) {
        return FlowNetInterface::wait_for_services();
    }

    return service_started;
    
}

bool FlowNetInterface::wait_for_services(ros::Duration timeout) {
    return ros::service::waitForService("flownet/analyse_image", timeout);
}    


bool FlowNetInterface::analyse(const cv::Mat& current_image,const cv::Mat& previous_image, cv::Mat& dst, cv::Mat& viz) {

    if (!service_started) {
        return false;
    }

    sensor_msgs::ImagePtr current_image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", current_image).toImageMsg();
    sensor_msgs::ImagePtr previous_image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", previous_image).toImageMsg();

    flow_net::FlowNet srv;
    srv.request.previous_image = *previous_image_msg;
    srv.request.current_image = *current_image_msg;

    if(client.call(srv)) {

        if (srv.response.success) {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(srv.response.output_image, sensor_msgs::image_encodings::TYPE_32FC2);
            dst = cv_ptr->image;

            cv_ptr = cv_bridge::toCvCopy(srv.response.output_viz, sensor_msgs::image_encodings::RGB8);
            viz = cv_ptr->image;


            return true;
        }
        else {
            ROS_ERROR_STREAM("Flow net service returned failed success");
            return false;
        }
        
    }
    else {
        ROS_ERROR_STREAM("Failed to call flow net service");
        return false;
    }




}