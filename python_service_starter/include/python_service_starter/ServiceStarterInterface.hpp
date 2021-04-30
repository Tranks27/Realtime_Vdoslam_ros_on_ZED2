#ifndef _PYTHON_SERVICE_STARTER_INTERFACE
#define _PYTHON_SERVICE_STARTER_INTERFACE

#include <ros/ros.h>
#include <functional>
#include <opencv2/opencv.hpp>


typedef std::function<void(const cv::Mat&, cv::Mat&)> PreprocessorFunc;

class ServiceStarterInterface {

    public:
        ServiceStarterInterface(ros::NodeHandle& _nh) :
            nh(_nh),
            service_started(false) {
                preprocessor = boost::bind(&ServiceStarterInterface::base_func, this, _1, _2);
            }

        /**
         * @brief Starts the service requested
         * 
         * @return true 
         * @return false 
         */
        virtual bool start_service(bool wait_for_services = true) = 0;

        /**
         * @brief Attaches a function to the inteface that will be called prior to the analyse 
         * function being called. This allows any minor preprocessing steps to be completed
         * before the image is parsed. The function is of the form void func(const cv::Mat& input, cv::Mat& ouput),
         * where output will be given to the relevant analyse function. 
         * 
         * @param func std::function<void(const cv::Mat&, cv::Mat&)>
         */
        void attach_preprocessor(PreprocessorFunc&& func) {
            preprocessor = func;
        }


        /**
         * @brief Waits for the specific services this interface will interact with
         * 
         * @param wait_for_services if true the function should use the wait for services function to wait for the 
         * appropiate services to be ready
         * @return true 
         * @return false 
         */
        static bool wait_for_services(ros::Duration timeout = ros::Duration(-1));

        std::function<void(const cv::Mat&, cv::Mat&)> preprocessor;

    protected:
        ros::NodeHandle nh;

        //the actual call to the monp depth network
        ros::ServiceClient client;
        ros::ServiceClient start_client;
        bool service_started;

    private:

        void base_func(const cv::Mat& src, cv::Mat& dst) {
            dst = src;
        }


        



};


#endif