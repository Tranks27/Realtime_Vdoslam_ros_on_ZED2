#include "visualizer/RosVisualizer.hpp"
#include "VdoSlamMsgInterface.hpp"

#include <opencv2/opencv.hpp>
#include <realtime_vdo_slam/VdoSlamScene.h>
#include <realtime_vdo_slam/VdoSceneObject.h>
#include <vdo_slam/visualizer/visualizer_2D.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <string>

#include "utils/RosUtils.hpp"
// #include "VdoSlamMsgInterface.hpp"



namespace VDO_SLAM {


    RosVisualizer::RosVisualizer(VisualizerParamsPtr& params_)
        :   Visualizer2D(params_),
            nh("ros_visualizer"),
            image_transport(nh),
            listener(tf_buffer) {


            nh.getParam("/ros_vdo_slam/use_ros_time", use_ros_time);

            //create mems for async callback queues
            vdo_scene_queue_ptr = std::make_shared<ros::CallbackQueue>();
            publish_queue_ptr = std::make_shared<ros::CallbackQueue>();

            async_manager = std::make_shared<RosAsyncManager>(publish_queue_ptr);


            //create async subscriber to our ros scene
            static constexpr size_t kMaxSceneQueueSize = 1000u;
            ros::SubscribeOptions vdo_slam_subscriber_options =
                ros::SubscribeOptions::create<realtime_vdo_slam::VdoSlamScene>(
                    "/vdoslam/output/scene",
                    kMaxSceneQueueSize,
                    boost::bind(&RosVisualizer::slam_scene_callback, this, _1),
                    ros::VoidPtr(),
                    vdo_scene_queue_ptr.get());

            vdo_slam_subscriber_options.transport_hints.tcpNoDelay(true);
            slam_scene_sub = nh.subscribe(vdo_slam_subscriber_options);


            ros::SubscribeOptions vdo_slam_map_subscriber_options =
                ros::SubscribeOptions::create<realtime_vdo_slam::VdoSlamMap>(
                    "/vdoslam/output/map",
                    kMaxSceneQueueSize,
                    boost::bind(&RosVisualizer::reconstruct_scenes_callback, this, _1),
                    ros::VoidPtr(),
                    vdo_scene_queue_ptr.get());

            vdo_slam_map_subscriber_options.transport_hints.tcpNoDelay(true);
            slam_map_sub = nh.subscribe(vdo_slam_map_subscriber_options);

            //create the actual async spinner to listen to the vdoslam/output/scene topic
            static constexpr size_t kSceneSpinnerThreads = 2u;
            async_spinner_scene =
                std::make_unique<ros::AsyncSpinner>(kSceneSpinnerThreads, vdo_scene_queue_ptr.get());

            //create all publishers that will use publish_queue_ptr (rather than the global queue)
            //they will eventually be attached to async_spinner_publish
            async_manager->create<visualization_msgs::MarkerArray>("/vdoslam/output/3dscene",
                    slam_scene_3d_pub, nh);

            async_manager->create<nav_msgs::Odometry>("/vdoslam/output/odom",
                    odom_pub, nh);

            async_manager->create<sensor_msgs::Image>("/vdoslam/output/bounding_box_image",
                bounding_box_pub, image_transport);

            async_manager->create<sensor_msgs::Image>("/vdoslam/output/object_point_image",
                object_track_pub, image_transport);

            static constexpr size_t kPublishSpinnerThreads = 2u;
            //create the ros spinner that is responsible for publishing all visualization messages
            async_spinner_publish =
                std::make_unique<ros::AsyncSpinner>(kPublishSpinnerThreads, publish_queue_ptr.get());


            nh.getParam("/vdo_pipeline/visualizer/odometry_ground_truth_topic", odom_gt_topic);
            nh.getParam("/vdo_pipeline/visualizer/gps_topic", gps_topic);


            //Getting Frame ID's
            nh.getParam("/vdo_pipeline/map_frame_id", map_frame);
            nh.getParam("/vdo_pipeline/odom_frame_id", odom_frame);
            nh.getParam("/vdo_pipeline/base_link_frame_id", base_frame);

            if(gt_odom_in_use()) {
                odom_gt_sub = nh.subscribe<nav_msgs::Odometry>(odom_gt_topic, 100, &RosVisualizer::odom_gt_callback, this);
            }

            if(gps_in_use()) {
                gps_sub = nh.subscribe<sensor_msgs::NavSatFix>(gps_topic, 100, &RosVisualizer::gps_callback, this);
            }


            nav_msgs::Odometry odom;
            odom.pose.pose.position.x = 0;
            odom.pose.pose.position.y = 0;
            odom.pose.pose.position.z = 0;

            odom.pose.pose.orientation.x = 0;
            odom.pose.pose.orientation.y = 0;
            odom.pose.pose.orientation.z = 0;
            odom.pose.pose.orientation.w = 1;

            //setting map frame and odom frame to be the same
            VDO_SLAM::utils::publish_static_tf(odom, map_frame, odom_frame);

            //setting base link starting point to be the same as odom
            VDO_SLAM::utils::publish_static_tf(odom, odom_frame, base_frame);

            display = cv::Mat::zeros(800, 800, CV_8UC3);

            ROS_INFO_STREAM("Created VDO_SLAM visualizer");

    }

    RosVisualizer::~RosVisualizer() {
        slam_scene_queue.shutdown();
        ROS_INFO_STREAM("Shutting down visualizer with " << slam_scene_queue.size());

        if(scene_spinner_started) {
            async_spinner_scene->stop();
        }
        if (publish_spinner_started) {
            async_spinner_publish->stop();
        }
    }

    void RosVisualizer::connect_handler(RosVizualizerSpinHandler& handler) {
        handler = std::async(std::launch::async,
                   &VDO_SLAM::RosVisualizer::spin_viz,
                   this);
    }

    bool RosVisualizer::spin_viz() {

        async_spinner_scene->start();
        scene_spinner_started = true;
        ROS_INFO_STREAM("Async Scene spinner started");

        async_spinner_publish->start();
        publish_spinner_started = true;
        ROS_INFO_STREAM("Async Publish spinner started");

        return true;

    }

    void RosVisualizer::slam_scene_callback(const realtime_vdo_slam::VdoSlamSceneConstPtr& slam_scene) {
        realtime_vdo_slam::VdoSlamSceneConstPtr slam_scene_ptr = slam_scene;
        SlamScenePtr scene = Scene::create<realtime_vdo_slam::VdoSlamSceneConstPtr>(slam_scene_ptr);
        VisualizerOutputPtr viz_output = spinOnce(scene);

        // publish_bounding_box_mat(viz_output);
        // publish_display_mat(viz_output);

    }

    void RosVisualizer::reconstruct_scenes_callback(const realtime_vdo_slam::VdoSlamMapConstPtr& map) {
        // ROS_INFO_STREAM("Recieved slam map updated");

        // //reset display
        // display = cv::Mat::zeros(800, 800, CV_8UC3);
        // for(const realtime_vdo_slam::VdoSlamScene& scene: map->scenes) {
        //     boost::shared_ptr<realtime_vdo_slam::VdoSlamScene> slam_scene_ptr = boost::make_shared<realtime_vdo_slam::VdoSlamScene>(scene);
        //     update_spin(slam_scene_ptr);
        // }
    }


    void RosVisualizer::odom_gt_callback(const nav_msgs::OdometryConstPtr& msg) {
        nav_msgs::Odometry odom_msg = *msg;

        if(use_ros_time) {
            //overwrite message time
            odom_msg.header.stamp = ros::Time::now();
        }

        VDO_SLAM::Odometry odom = VDO_SLAM::Odometry::create<nav_msgs::Odometry>(odom_msg);
        update_gt_odom(odom);
    }

    void RosVisualizer::gps_callback(const sensor_msgs::NavSatFixConstPtr& msg) {
        if (is_first_gps) {
            double easting, northing;
            std::string zone;

            double lat = msg->latitude;
            double longitude = msg->longitude;

            utils::LLtoUTM(lat, longitude, northing, easting, zone);
            ROS_INFO_STREAM("Updating map topic with easting: " << easting << " northing: " << northing);

            nav_msgs::Odometry odom;
            odom.pose.pose.position.x = easting;
            odom.pose.pose.position.y = northing;
            odom.pose.pose.position.z = 0;

            odom.pose.pose.orientation.x = 0;
            odom.pose.pose.orientation.y = 0;
            odom.pose.pose.orientation.z = 0;
            odom.pose.pose.orientation.w = 1;
            is_first_gps = false;

            VDO_SLAM::utils::publish_static_tf(odom, map_frame, odom_frame);

        }
    }

    ros::Publisher RosVisualizer::create_viz_pub(ros::NodeHandle& nh) {
        return nh.advertise<realtime_vdo_slam::VdoSlamScene>("/vdoslam/output/scene", 10);
    }

    ros::Publisher RosVisualizer::create_map_update_pub(ros::NodeHandle& nh) {
        return nh.advertise<realtime_vdo_slam::VdoSlamMap>("/vdoslam/output/map", 10);
    }

    void RosVisualizer::publish_odom(const realtime_vdo_slam::VdoSlamScenePtr& slam_scene) {
        nav_msgs::Odometry odom;
        utils::geometry_to_odom(slam_scene->camera_pose, slam_scene->camera_twist, 
            slam_scene->header.stamp, odom_frame, base_frame, odom);

        odom_pub.publish(odom);

        utils::publish_static_tf(odom, odom_frame, base_frame);

    }

    // void RosVisualizer::publish_3D_viz(const realtime_vdo_slam::VdoSlamScenePtr& slam_scene) {
    //     visualization_msgs::MarkerArray marker_array;
    //     int i = 0;
    //     for (realtime_vdo_slam::VdoSceneObject& scene_object : slam_scene->objects) {
    //         visualization_msgs::Marker marker;
    //         marker.header.frame_id = odom_frame;
    //         marker.header.stamp = slam_scene->header.stamp;
    //         marker.text = scene_object.label;
    //         marker.ns = "vdoslam";
    //         marker.id = i;
    //         // marker.type = visualization_msgs::Marker::SPHERE;
    //         marker.type = visualization_msgs::Marker::CUBE;
    //         marker.action = visualization_msgs::Marker::ADD;
    //         marker.pose.position.x = scene_object.pose.position.x;
    //         marker.pose.position.y = scene_object.pose.position.y;
    //         marker.pose.position.z = 0;

    //         marker.pose.orientation.x = 0.0;
    //         marker.pose.orientation.y = 0.0;
    //         marker.pose.orientation.z = 0.0;
    //         marker.pose.orientation.w = 1.0;
    //         marker.scale.x = 0.5;
    //         marker.scale.y = 0.5;
    //         marker.scale.z = 2;
    //         marker.color.a = 1.0; // Don't forget to set the alpha!
    //         marker.color.r = 0.0;
    //         marker.color.g = 1.0;
    //         marker.color.b = 0.0;
    //         marker.lifetime = ros::Duration(2.0);
    //         // marker.text = scene_object.label;
    //         marker.text = scene_object.tracking_id;
    //         marker_array.markers.push_back(marker);
    //         i++;
    //     }

    //     slam_scene_3d_pub.publish(marker_array);

    // }

    void RosVisualizer::publish_display_mat(const VisualizerOutputPtr& viz_output) {
        VisualizerOutput2DPtr viz_output_2D = std::dynamic_pointer_cast<VisualizerOutput2D>(viz_output);

        if(object_track_pub.getNumSubscribers() > 0) {
            sensor_msgs::Image img_msg;
            utils::mat_to_image_msg(img_msg, viz_output_2D->object_point_display_, sensor_msgs::image_encodings::BGR8);
            object_track_pub.publish(img_msg);
        }

    }


    void RosVisualizer::publish_bounding_box_mat(const VisualizerOutputPtr& viz_output) {
        VisualizerOutput2DPtr viz_output_2D = std::dynamic_pointer_cast<VisualizerOutput2D>(viz_output);

        if (bounding_box_pub.getNumSubscribers() > 0) {
            sensor_msgs::Image img_msg;
            utils::mat_to_image_msg(img_msg, viz_output_2D->bounding_box_display_, sensor_msgs::image_encodings::RGB8);
            bounding_box_pub.publish(img_msg);
        }

    }







}