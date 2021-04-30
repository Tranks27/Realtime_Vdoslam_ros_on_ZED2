#ifndef _REALTIME_VDO_SLAM
#define _REALTIME_VDO_SLAM


#include <ros/ros.h>
#include <nodelet/loader.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <image_transport/image_transport.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <iostream>
#include <stdio.h>
#include <nav_msgs/Odometry.h>


#include <my_realtime_vdo_slam/VdoInput.h>
#include <my_realtime_vdo_slam/VdoSlamScene.h>
#include <vdo_slam/Scene.h>
#include <vdo_slam/System.h>
#include "utils/ThreadedQueue.hpp"
#include "VdoSlamInput.hpp"
#include "visualizer/RosVisualizer.hpp"
#include "tracking/SemanticTracker.hpp"


#include <string>
#include <vector>
#include <algorithm>
#include <map>
#include <memory>
#include <queue>
#include <mutex>
#include <thread>

typedef const sensor_msgs::ImageConstPtr& ImageConst;

class RosVdoSlam{
    public:
        /**
         * @brief Runs the Vdo Slam algorithm in the ROS environment. VDO requires input as RGB image, semantic mask,
         * optical flow and depth map. The class synchronizes the input messages using message_filter::TimeSynchronizer
         * and parses the images to the VDO_SLAM::System class. As the time for computation is significantly greater
         * than the frequency of any reayltime data stream a threaded queue is used to parse input VdoSlamInput to the 
         * VDO_SLAM::System. 
         * 
         * @param n 
         */
        RosVdoSlam(ros::NodeHandle& n);
        ~RosVdoSlam();

        /**
         * @brief Synchronized message callback to obtain the data needed for the the VDO_SLAM::System. The callback listens 
         * to the topics. Becuase this information comes from the vdo_slam_preprocesing node (which uses the parent namespace /vdoslam/input/)
         * we remap these topic to the desired topics here (see the launch file).
         * - raw_image: /camera/rgb/image_raw
         * - mask: /camera/mask/image_raw
         * - flow: /camera/flow/image_raw
         * - depth /camera/depth/image_raw
         * 
         * This callback creates as pointer to a VdoSlamInput which contains all the input necessary to the VDO_SLAM algorithm and adds 
         * it to the worker queue. 
         * 
         * @param vdo_input realtime_vdo_slam::VdoInput
         */
        void vdo_input_callback(const my_realtime_vdo_slam::VdoInputConstPtr& vdo_input);

    private:
        ros::NodeHandle handle;

        /**
         * @brief Constructs the desired slam system with parameters and configuration defined
         * in the launch file.
         * 
         * @param nh Ros::NodeHandle
         * @return std::shared_ptr<VDO_SLAM::System>
         */
        std::shared_ptr<VDO_SLAM::System> construct_slam_system(ros::NodeHandle& nh);

        /**
         * @brief Worker thread for the VDO_SLAM queue. Will continue as long as ros::okay() returns true. 
         * 
         */
        void vdo_worker();

        

        ////////////////////// Variables
        int global_optim_trigger;

        //VdoSlam
        int viz_rate;
        bool use_viz;

        ros::Publisher scene_pub;
        ros::Publisher map_pub;
        VDO_SLAM::RosVisualizerPtr ros_viz;
        VDO_SLAM::RosVizualizerSpinHandler ros_viz_handler;

        VDO_SLAM::SlamScenePtr vdo_scene;

        //this will grow unfiltered with time - could have a cap on size?
        std::vector<VDO_SLAM::SlamScenePtr> vdo_scene_vector;

        cv::Mat image_trajectory;
        std::shared_ptr<VDO_SLAM::System> slam_system;

        //VdoSlam input
        ThreadsafeQueue<VDO_SLAM::VdoSlamInputPtr> vdo_input_queue;
        // std::mutex queue_mutex;
        std::thread vdo_worker_thread;

        typedef int TrackingID;
        typedef std::string ClassType;

        //map to hold tracking ID's to their class label
        std::map<TrackingID, ClassType> tracking_class_map;

        ros::Subscriber vdo_input_sub;


        ros::Time current_time;
        ros::Time previous_time;

        SemanticTracker tracker;
};












#endif