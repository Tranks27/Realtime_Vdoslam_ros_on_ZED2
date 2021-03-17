#include <ros/ros.h>
#include <string>
#include <thread>
#include <sstream>
#include <memory>
#include <ros/package.h>

#include "PythonServiceController.hpp"



PythonServiceController::PythonServiceController(ros::NodeHandle& _nh) :
    nh(_nh),
    flow_status(false),
    mask_rcnn_status(false),
    run_output_threads(true)
{
    std::string path;
    path = ros::package::getPath("mono_depth_2");
    python_mono_depth_full_path = path + std::string("/scripts/mono_depth_rospy.py");

    path = ros::package::getPath("mask_rcnn");
    python_mask_rcnn_full_path = path + std::string("/scripts/mask_rcnn_rospy.py");

    path = ros::package::getPath("flow_net");
    python_flow_net_full_path = path + std::string("/scripts/flow_net_rospy.py");


    start_flow_net_service = nh.advertiseService("start_flow_net", &PythonServiceController::init_flow_net, this);
    start_mask_rcnn_service = nh.advertiseService("start_mask_rcnn", &PythonServiceController::init_mask_rcnn, this);
    start_mono_depth_service = nh.advertiseService("start_mono_depth", &PythonServiceController::init_mono_depth, this);
    // handler.param<std::string>("/realtime_vdo_slam/topic_prefix", topic_prefix, "/gmsl/");
}

PythonServiceController::~PythonServiceController() {}

bool PythonServiceController::init_flow_net(python_service_starter::StartFlowNet::Request& request, python_service_starter::StartFlowNet::Response& response) {
    if (request.start) {
        flow_net_service_starter = std::make_unique<ServiceStarter>(python_flow_net_full_path);
        
        if(flow_net_service_starter->start_program()) {
            ROS_INFO_STREAM("program " << flow_net_service_starter->get_program_name() << " started");
            return true;
        }
        else {
            ROS_WARN_STREAM("program " << flow_net_service_starter->get_program_name() << " failed");
            return false;
        }
        
    }
    return true;
}



bool PythonServiceController::init_mask_rcnn(python_service_starter::StartMaskRcnn::Request& request, python_service_starter::StartMaskRcnn::Response& response) {
    if (request.start) {
        mask_rcnn_service_starter = std::make_unique<ServiceStarter>(python_mask_rcnn_full_path);
        
        if(mask_rcnn_service_starter->start_program()) {
            ROS_INFO_STREAM("program " << mask_rcnn_service_starter->get_program_name() << " started");
            return true;
        }
        else {
            ROS_WARN_STREAM("program " << mask_rcnn_service_starter->get_program_name() << " failed");
            return false;
        }
        
    }
    return true;
}

bool PythonServiceController::init_mono_depth(python_service_starter::StartMonoDepth::Request& request, python_service_starter::StartMonoDepth::Response& response) {
    if (request.start) {
        mono_depth_service_starter = std::make_unique<ServiceStarter>(python_mono_depth_full_path);
        
        if(mono_depth_service_starter->start_program()) {
            ROS_INFO_STREAM("program " << mono_depth_service_starter->get_program_name() << " started");
            return true;
        }
        else {
            ROS_WARN_STREAM("program " << mono_depth_service_starter->get_program_name() << " failed");
            return false;
        }
        
    }
    return true;
}


bool PythonServiceController::shutdown_services() {
    if (flow_net_service_starter && flow_net_service_starter->get_status()) {
        flow_net_service_starter->shutdown();
    }
    if (mask_rcnn_service_starter && mask_rcnn_service_starter->get_status()) {
        mask_rcnn_service_starter->shutdown();
    }

    if (mono_depth_service_starter && mono_depth_service_starter->get_status()) {
        mono_depth_service_starter->shutdown();
    }
    return true;
}

