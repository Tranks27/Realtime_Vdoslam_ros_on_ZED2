#include <ros/ros.h>
#include <string>
#include <thread>
#include <sstream>
#include <memory>
#include <signal.h>

#include "PythonServiceController.hpp"

PythonServiceControllerPtr service_starter;

void shutdown_signal_handler(int sig) {
    ROS_INFO_STREAM("shutting down python services");
    service_starter->shutdown_services();
    ros::shutdown();
}



int main(int argc, char ** argv) {

    ros::init(argc, argv, "python_service_starter", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;

    signal(SIGINT, shutdown_signal_handler);

    service_starter = std::make_unique<PythonServiceController>(nh);

    ros::spin();

}