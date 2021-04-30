#ifndef _ROS_VDO_VISUALIZER
#define _ROS_VDO_VISUALIZER

#include <mutex>
#include <future>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/NavSatFix.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <image_transport/image_transport.h>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <nav_msgs/Odometry.h>
#include <image_transport/image_transport.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <opencv2/opencv.hpp>
#include <my_realtime_vdo_slam/VdoSlamScene.h>
#include <my_realtime_vdo_slam/VdoSlamMap.h>
#include <vdo_slam/visualizer/colour.h>
#include <vdo_slam/visualizer/visualizer_2D.h>
#include <vdo_slam/visualizer/visualizer_params.h>

#include "utils/ThreadedQueue.hpp"
#include "RosAsyncManager.hpp"

namespace VDO_SLAM {

    // /**
    //  * @brief Overlays an image with all information from a Slam Scene message.
    //  * This includes bounding box, class, tracking ID, pose and velocity
    //  * 
    //  * @param image rgb image to overlay
    //  * @param slam_scene slam scene to details
    //  * @return cv::Mat overlayed image
    //  */
    // cv::Mat overlay_scene_image(const cv::Mat& image, const my_realtime_vdo_slam::VdoSlamScenePtr& slam_scene);

    typedef std::future<bool> RosVizualizerSpinHandler;


    class RosVisualizer : public Visualizer2D {

        public:
            /**
             * @brief Construct a new Ros Visualizer object
             * 
             */
            RosVisualizer(VisualizerParamsPtr& params_);
            ~RosVisualizer();

            /**
             * @brief Connects the handler to a function asynchronously.
             * 
             * @param handler RosVizualizerSpinHandler
             */
            void connect_handler(RosVizualizerSpinHandler& handler);

            /**
             * @brief Spins the vizualizer asynchronously at a given rate. Upon execution all vdo slam scene messages
             * in the callback queue will be displayed and published. We use asynchronous ros callback queues for each vdoscene message
             * and for all publishers. This is required to output the visualization at a high rate without taking up time for the VDO-SLAM
             * algorithm to run. Each callback queue is given its own asynch spinner.
             * 
             * @return true 
             * @return false 
             */
            bool spin_viz();

            /**
             * @brief Asychronously subscribes to to the /vdoslam/output/scene topic, before adding the slam scene to a
             * thread safe queue to be used by publishing spinners.
             * 
             * @param slam_scene my_realtime_vdo_slam::VdoSlamSceneConstPtr&
             */
            void slam_scene_callback(const my_realtime_vdo_slam::VdoSlamSceneConstPtr& slam_scene);
            /**
             * @brief Asychronously subscribes to to the /vdoslam/output/map topic which contains a list of all VdoSlamScenes
             * after graph optimization. Upon callback scenes will be redrawn with the updated pose
             * 
             * @param slam_scene my_realtime_vdo_slam::VdoSlamMapConstPtr&
             */
            void reconstruct_scenes_callback(const my_realtime_vdo_slam::VdoSlamMapConstPtr& map);

            /**
             * @brief Subscribes to nav_msgs::Odometry messages that will be used for ground truth. Listenes to the /ros_vdo/odometry_ground_truth_topic
             * defined in the launch file. (see VDO_SLAM::Utils in RosScene.cpp) It will offset the odom gt bt some amount 
             * such that the gt odom starts at [0,0,0][0,0,0,1] even if the ROS information playing does not start centerd at the world 
             * coordinates. This purpose of this is to start the VDO_SLAM odom and the gt odom at the same place so we can compare them visually.
             * 
             * If odom started at the world frame, this would not be necessary as the VDO_SLAM odom starts at [0,0,0][0,0,0,1].
             * Internally, this will add the gt square to the display mat object (or should this be done by a function call?)
             * 
             * @param msg 
             */
            void odom_gt_callback(const nav_msgs::OdometryConstPtr& msg);

            /**
             * @brief Subscribes to sensor_msgs::NavSatFix if availble. Listens to the /ros_vdo/gps_topic defined in the launch file. Once a message
             * comes in we use this to set the map frame relative to the odom frame. Currently not used for data fusion. 
             * 
             * @param msg 
             */
            void gps_callback(const sensor_msgs::NavSatFixConstPtr& msg);

            /**
             * @brief Static function to create a ros publisher that publishes to the topic that the visualizer expects data on.
             * Publishes a my_realtime_vdo_slam::VdoSlamScene message.
             * 
             * @param nh 
             * @return ros::Publisher 
             */
            static ros::Publisher create_viz_pub(ros::NodeHandle& nh);

            /**
             * @brief Static function to create a ros publisher that publishes to the topic that the visualizer expects data on
             * that contains an updated map
             * Publishes a my_realtime_vdo_slam::VdoSlamMap message.
             * 
             * @param nh 
             * @return ros::Publisher 
             */
            static ros::Publisher create_map_update_pub(ros::NodeHandle& nh);


        private:

            /**
             * @brief Publishes the latest odometry from the VDO slam algorithm. If tf transforms are available,
             * the tf tree will also be updated.
             * 
             *  @param slam_scene my_realtime_vdo_slam::VdoSlamScenePtr&
             */
            void publish_odom(const my_realtime_vdo_slam::VdoSlamScenePtr& slam_scene);


            /**
             * @brief Publishes and visualises the birdseye view of the scene. Uses update_display_mat() to update the 
             * display mat
             * 
             * @param scene const VisualizerOutputPtr&
             */
            void publish_display_mat(const VisualizerOutputPtr& viz_output);

            /**
             * @brief Publishes the slam image with bounding box, class and velocity information draw
             * over the image. Makes a call to overlay_scene_image
             * 
             * @param scene const VisualizerOutputPtr&
             */
            void publish_bounding_box_mat(const VisualizerOutputPtr& viz_output);

            inline bool gt_odom_in_use() {return !odom_gt_topic.empty();}

            inline bool gps_in_use() {return !gps_topic.empty();}



            ros::NodeHandle nh;
            int spin_rate;

            VDO_SLAM::RosAsyncPublisherPtr async_manager;


            RosCallbackQueuePtr vdo_scene_queue_ptr;
            //used for subscribing to the vdo scene topic
            std::unique_ptr<ros::AsyncSpinner> async_spinner_scene;
            bool scene_spinner_started = false;


            RosCallbackQueuePtr publish_queue_ptr;
            //used for subscribing to the vdo scene topic
            std::unique_ptr<ros::AsyncSpinner> async_spinner_publish;
            bool publish_spinner_started = false;

            //subscribe to VdoSlamScene msg
            ros::Subscriber slam_scene_sub;
            //subscribe to VdoSlamMap msg
            ros::Subscriber slam_map_sub;
            
            //publishes the slam scene as markers for RVIZ
            ros::Publisher slam_scene_3d_pub;

            //publish camera pose from VDO as odom
            ros::Publisher odom_pub;

            image_transport::ImageTransport image_transport;
            image_transport::Publisher bounding_box_pub;

            bool display_window;
            cv::Mat display; //static and dynamic object track
            image_transport::Publisher object_track_pub;

            //Thread safe queue for VdoSlamScene's
            ThreadsafeQueue<my_realtime_vdo_slam::VdoSlamScenePtr> slam_scene_queue;


            // subcrsibes to gt odom if exists. Odom gt topic defined in vdo_slam launch file
            //Info needed for gt odom
            bool is_first_odom = true;
            float odom_x_offset;
            float odom_y_offset;
            std::string odom_gt_topic;
            ros::Subscriber odom_gt_sub;
            nav_msgs::Odometry gt_odom;

            bool is_first_gps = true;
            std::string gps_topic;
            ros::Subscriber gps_sub;
            sensor_msgs::NavSatFix gps_msg;


            // std::mutex display_mutex;

            tf2_ros::TransformBroadcaster broadcaster;
            tf2_ros::Buffer tf_buffer;
            tf2_ros::TransformListener listener;

            
            //frames for tf tree to publish on
            std::string map_frame;
            std::string odom_frame;
            std::string base_frame;
            


            static int vis_count; //used for marker visualisation
    };

    typedef std::shared_ptr<RosVisualizer> RosVisualizerPtr;
    typedef std::unique_ptr<RosVisualizer> RosVisualizerUniquePtr;


}


#endif