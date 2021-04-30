#ifndef _PYTHON_SERVICE_STARTER_CONTROLLER
#define _PYTHON_SERVICE_STARTER_CONTROLLER


#include <ros/ros.h>
#include <string>
#include <thread>
#include <sstream>
#include <memory>

#include "python_service_starter/StartFlowNet.h"
#include "python_service_starter/StartMaskRcnn.h"
#include "python_service_starter/StartMonoDepth.h"
#include "python_service_starter/StartMidasDepth.h"

#include "ServiceStarter.hpp"


class PythonServiceController {

    public:
        PythonServiceController(ros::NodeHandle& _nh);
        ~PythonServiceController();

        bool init_flow_net(python_service_starter::StartFlowNet::Request& request, python_service_starter::StartFlowNet::Response& response);
        bool init_mask_rcnn(python_service_starter::StartMaskRcnn::Request& request, python_service_starter::StartMaskRcnn::Response& response);
        bool init_mono_depth(python_service_starter::StartMonoDepth::Request& request, python_service_starter::StartMonoDepth::Response& response);
        bool init_midas_depth(python_service_starter::StartMidasDepth::Request& request, python_service_starter::StartMidasDepth::Response& response);
        

        bool shutdown_services();

  
    private:
        bool flow_status;
        bool mask_rcnn_status;
        // PipeCommsManager flow_net_communication;

        bool run_output_threads;

        ros::NodeHandle nh;
        ros::ServiceServer start_flow_net_service;
        ros::ServiceServer start_mask_rcnn_service;
        ros::ServiceServer start_mono_depth_service;
        ros::ServiceServer start_midas_depth_service;

        


        std::string python_flow_net_full_path;
        std::string python_mask_rcnn_full_path;
        std::string python_mono_depth_full_path;
        std::string python_midas_depth_full_path;

        std::unique_ptr<ServiceStarter> flow_net_service_starter;
        std::unique_ptr<ServiceStarter> mask_rcnn_service_starter;
        std::unique_ptr<ServiceStarter> mono_depth_service_starter;
        std::unique_ptr<ServiceStarter> midas_depth_service_starter;

};

typedef std::unique_ptr<PythonServiceController> PythonServiceControllerPtr;

#endif

