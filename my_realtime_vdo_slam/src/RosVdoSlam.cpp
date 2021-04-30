#include <vdo_slam/utils/Types.h>
#include <vdo_slam/utils/VdoUtils.h>
#include <vdo_slam/System.h>
#include <vdo_slam/Scene.h>
#include <vdo_slam/visualizer/visualizer_params.h>

#include <ros/package.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Odometry.h>
#include <tf2/transform_datatypes.h>
#include <tf2/convert.h>
#include <memory>
#include <future>

#include "RosVdoSlam.hpp"
#include <my_realtime_vdo_slam/VdoInput.h>
#include "VdoSlamInput.hpp"
#include "visualizer/RosVisualizer.hpp"
#include "utils/RosUtils.hpp"
#include "VdoSlamMsgInterface.hpp"





using namespace VDO_SLAM;

RosVdoSlam::RosVdoSlam(ros::NodeHandle& n):
    handle(n){
        handle.getParam("/ros_vdo_slam/use_viz", use_viz);
        handle.getParam("/ros_vdo_slam/viz_rate", viz_rate);


        handle.getParam("/ros_vdo_slam/optimization_trigger_frame", global_optim_trigger);
        ROS_INFO_STREAM("Global Optimization Trigger at frame id: " << global_optim_trigger);

        
        if (use_viz) {
            VisualizerParamsPtr viz_params = std::make_shared<VisualizerParams>();
            viz_params->classes_filepath = "/home/tranks/testing_ws/src/VDO_SLAM/include/vdo_slam/visualizer/classes.csv";
            handle.getParam("/vdo_pipeline/visualizer/display_window", viz_params->display_window);

            std::make_shared<RosVisualizer>(viz_params);
            // ros_viz = std::make_shared<RosVisualizer>(viz_params);
            // ros_viz->connect_handler(ros_viz_handler);
        }





        vdo_worker_thread = std::thread(&RosVdoSlam::vdo_worker, this);
    }

RosVdoSlam::~RosVdoSlam(){
    if(vdo_worker_thread.joinable()){
        vdo_worker_thread.join();
    }
}

std::shared_ptr<VDO_SLAM::System> RosVdoSlam::construct_slam_system(ros::NodeHandle& nh){

}

void RosVdoSlam::vdo_input_callback(const my_realtime_vdo_slam::VdoInputConstPtr& vdo_input){

}


void RosVdoSlam::vdo_worker(){
    std::unique_ptr<VDO_SLAM::Scene> scene;

    while(ros::ok() && !vdo_input_queue.isShutdown()){

    }
}































