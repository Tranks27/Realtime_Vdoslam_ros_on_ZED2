#ifndef _PYTHON_SERVICE_STARTER_INTERFACE
#define _PYTHON_SERVICE_STARTER_INTERFACE

#include <ros/ros.h>
#include <functional>


class ServiceStarterInterface {

    public:
        ServiceStarterInterface(ros::NodeHandle& _nh) :
            nh(_nh),
            service_started(false) {}

        /**
         * @brief Starts the service requested
         * 
         * @return true 
         * @return false 
         */
        virtual bool start_service(bool wait_for_services = true) = 0;

        /**
         * @brief Waits for the specific services this interface will interact with
         * 
         * @param wait_for_services if true the function should use the wait for services function to wait for the 
         * appropiate services to be ready
         * @return true 
         * @return false 
         */
        static bool wait_for_services(ros::Duration timeout = ros::Duration(-1));

    protected:
        ros::NodeHandle nh;

        //the actual call to the monp depth network
        ros::ServiceClient client;
        ros::ServiceClient start_client;
        bool service_started;
        



};


#endif