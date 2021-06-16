#include "RosVdoSlam.hpp"
// #include "RosScene.hpp"
#include "VdoSlamMsgInterface.hpp"
#include "VdoSlamInput.hpp"
#include "CameraInformation.hpp"
#include "utils/RosUtils.hpp"
#include "visualizer/RosVisualizer.hpp"
#include <realtime_vdo_slam/VdoInput.h>

#include <vdo_slam/utils/Types.h>
#include <vdo_slam/utils/VdoUtils.h>
#include <vdo_slam/System.h>
#include <vdo_slam/Scene.h>
#include <vdo_slam/definitions.h>
#include <vdo_slam/visualizer/visualizer_params.h>

#include <ros/package.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Odometry.h>
#include <tf2/transform_datatypes.h>
#include <tf2/convert.h>
#include <memory>
#include <future>


using namespace VDO_SLAM;

RosVdoSlam::RosVdoSlam(ros::NodeHandle& n) :
    handle(n) {
        handle.getParam("/ros_vdo_slam/use_viz", use_viz);
        handle.getParam("/ros_vdo_slam/viz_rate", viz_rate);
        handle.getParam("/ros_vdo_slam/use_ros_time", use_ros_time);


        handle.getParam("/ros_vdo_slam/optimization_trigger_frame", global_optim_trigger);
        ROS_INFO_STREAM("Global Optimization Trigger at frame id: " << global_optim_trigger);

        if (use_viz) {
            VisualizerParamsPtr viz_params = std::make_shared<VisualizerParams>();
            viz_params->classes_filepath = std::string(__VDO_SLAM_DIR__) + "include/vdo_slam/visualizer/classes.csv";
            handle.getParam("/vdo_pipeline/visualizer/display_window", viz_params->display_window);


            ros_viz = std::make_shared<RosVisualizer>(viz_params);
            ros_viz->connect_handler(ros_viz_handler);
        }
        //TODO: should get proper previous time
        previous_time = ros::Time::now();
        image_trajectory = cv::Mat::zeros(800, 600, CV_8UC3);

        vdo_input_sub = handle.subscribe("/vdoslam/input/all", 100, &RosVdoSlam::vdo_input_callback, this);

        scene_pub = RosVisualizer::create_viz_pub(handle);
        map_pub = RosVisualizer::create_map_update_pub(handle);

        slam_system = construct_slam_system(handle);

        vdo_worker_thread = std::thread(&RosVdoSlam::vdo_worker, this);

    }

RosVdoSlam::~RosVdoSlam() {
    if (vdo_worker_thread.joinable()) {
        vdo_worker_thread.join();
    }
}


void RosVdoSlam::shutdown() {
    ROS_INFO_STREAM("Shutting down ROS VDO SLAM System");
    if (vdo_worker_thread.joinable()) {
        vdo_worker_thread.join();
    }

    ros_viz->shutdown();
    slam_system->shutdown();
}

std::shared_ptr<VDO_SLAM::System> RosVdoSlam::construct_slam_system(ros::NodeHandle& nh) {
    //check first where to load the params from - calibration file or launch file
    bool use_calibration_file;
    nh.getParam("/ros_vdo_slam/use_calibration_file", use_calibration_file);

    //load from calibration file
    if(use_calibration_file) {
        int sensor_mode;
        nh.getParam("/ros_vdo_slam/sensor_mode", sensor_mode);

        VDO_SLAM::eSensor sensor = VDO_SLAM::utils::param_to_sensor(sensor_mode);

        std::string calibration_file;
        nh.getParam("/ros_vdo_slam/calibration_file", calibration_file);

        std::string path = ros::package::getPath("realtime_vdo_slam");
        std::string vdo_slam_config_path = path + "/config/" + calibration_file;


        return std::make_shared<VDO_SLAM::System>(vdo_slam_config_path, sensor);
    }
    //load from params in launch
    else {
        VDO_SLAM::VdoParams params;
        //check if we should get instrincs from camera info topic
        bool use_camera_info_topic;
        nh.getParam("/ros_vdo_slam/use_camera_info_topic", use_camera_info_topic);

        //wait for topic defined in configuration file
        if (use_camera_info_topic) {
            std::string camera_info_topic = "/zed2/zed_node/rgb/camera_info";
            // nh.getParam("/ros_vdoslam/input_camera_info_topic", camera_info_topic);
            ROS_INFO_STREAM("Waiting for camera info topic: " << camera_info_topic);
            sensor_msgs::CameraInfoConstPtr info_ptr = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_info_topic);

            VDO_SLAM::CameraInformation cam_info(info_ptr);
            
            bool apply_undistortion;
            nh.getParam("/ros_vdo_slam/apply_undistortion", apply_undistortion);

            //if true -> we use the modified camera matrix P
            if(apply_undistortion) {
                //should be 3x3
                params.fx = cam_info.modified_camera_matrix.at<double>(0,0);
                params.cx = cam_info.modified_camera_matrix.at<double>(0,2);
                params.fy = cam_info.modified_camera_matrix.at<double>(1,1);
                params.cy = cam_info.modified_camera_matrix.at<double>(1,2);
            }
            //else use the original matrix K
            else {
                params.fx = cam_info.camera_matrix.at<double>(0,0);
                params.cx = cam_info.camera_matrix.at<double>(0,2);
                params.fy = cam_info.camera_matrix.at<double>(1,1);
                params.cy = cam_info.camera_matrix.at<double>(1,2);
            }
        }
        //load camera params from launch file
        else {
            ROS_WARN_STREAM("camera_info_topic is not being used");
            // nh.getParam("/ros_vdo_slam/fx", params.fx);
            // nh.getParam("/ros_vdo_slam/fy", params.fy);
            // nh.getParam("/ros_vdo_slam/cx", params.cx);
            // nh.getParam("/ros_vdo_slam/cy", params.cy);
            
        }

        //for now just set dist params to 0
        params.k1 = 0;
        params.k2 = 0;
        params.p1 = 0;
        params.p2 = 0;
        params.p3 = 0;

        //load rest of params from file
        nh.getParam("/ros_vdo_slam/width", params.width);
        nh.getParam("/ros_vdo_slam/height", params.height);

        nh.getParam("/ros_vdo_slam/fps", params.fps);
        // nh.getParam("/ros_vdo_slam/bf", params.bf);
        params.bf = params.fx * 0.12; // fx is always changing depending on camera_info topic, 0.12m is zed2 baseline

        nh.getParam("/ros_vdo_slam/RGB", params.RGB);
        nh.getParam("/ros_vdo_slam/data_code", params.data_code);

        int sensor_mode;
        nh.getParam("/ros_vdo_slam/sensor_mode", sensor_mode);

        params.sensor_type = VDO_SLAM::utils::param_to_sensor(sensor_mode);

        nh.getParam("/ros_vdo_slam/depth_map_factor", params.depth_map_factor);

        nh.getParam("/ros_vdo_slam/thdepth_bg", params.thdepth_bg);
        nh.getParam("/ros_vdo_slam/thdepth_obj", params.thdepth_obj);

        nh.getParam("/ros_vdo_slam/max_track_points_bg", params.max_track_points_bg);
        nh.getParam("/ros_vdo_slam/max_track_points_obj", params.max_track_points_obj);

        nh.getParam("/ros_vdo_slam/sf_mg_thresh", params.sf_mg_thresh);
        nh.getParam("/ros_vdo_slam/sf_ds_thresh", params.sf_ds_thresh);

        nh.getParam("/ros_vdo_slam/window_size", params.window_size);
        nh.getParam("/ros_vdo_slam/overlap_size", params.overlap_size);

        nh.getParam("/ros_vdo_slam/use_sample_feature", params.use_sample_feature);


        nh.getParam("/ros_vdo_slam/n_features", params.n_features);

        nh.getParam("/ros_vdo_slam/scale_factor", params.scale_factor);

        nh.getParam("/ros_vdo_slam/n_levels", params.n_levels);

        nh.getParam("/ros_vdo_slam/ini_th_fast", params.ini_th_fast);
        nh.getParam("/ros_vdo_slam/min_th_fast", params.min_th_fast);

        return std::make_shared<VDO_SLAM::System>(params);

    }


}

// void RosVdoSlam::vdo_input_callback(ImageConst raw_image, ImageConst mask, ImageConst flow, ImageConst depth) {
void RosVdoSlam::vdo_input_callback(const realtime_vdo_slam::VdoInputConstPtr& vdo_input) {
    //the actual time the image was craeted
    if(vdo_input->header.stamp.is_zero()) {
        ROS_ERROR_STREAM("VDO SLAM input time is zero");
    }

    if (use_ros_time) {
        current_time = ros::Time::now();
    }
    else {
        current_time = vdo_input->header.stamp;

    }
    ros::Duration diff = current_time - previous_time;
    //time should be in n seconds or seconds (or else?)
    double time_difference = diff.toSec();

    cv::Mat image, scene_flow_mat, mono_depth_mat, mask_rcnn_mat;
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(vdo_input->rgb, sensor_msgs::image_encodings::RGB8);
    image = cv_ptr->image;

    cv_ptr = cv_bridge::toCvCopy(vdo_input->mask, sensor_msgs::image_encodings::MONO8);
    mask_rcnn_mat = cv_ptr->image;

    cv_ptr = cv_bridge::toCvCopy(vdo_input->flow, sensor_msgs::image_encodings::TYPE_32FC2);
    scene_flow_mat = cv_ptr->image;

    cv_ptr = cv_bridge::toCvCopy(vdo_input->depth, sensor_msgs::image_encodings::MONO16);
    mono_depth_mat = cv_ptr->image;

    SemanticObjectVector semantic_objects = vdo_input->semantic_objects;


    std::shared_ptr<VdoSlamInput> input = std::make_shared<VdoSlamInput>(image,scene_flow_mat,
            mono_depth_mat, mask_rcnn_mat, semantic_objects, time_difference, current_time);

    //add the input to the thread queue so we can deal with it later
    // push_vdo_input(input);
    vdo_input_queue.push(input);
    previous_time = current_time;
    
}


void RosVdoSlam::merge_scene_semantics(SlamScenePtr& scene, const std::vector<mask_rcnn::SemanticObject>& semantic_objects) {
    std::vector<cv::Point2f> points;
    std::vector<vision_msgs::BoundingBox2D> bb;
    std::vector<VDO_SLAM::SceneObjectPtr> scene_objects = scene->get_scene_objects();
    for(VDO_SLAM::SceneObjectPtr& object : scene_objects) {
        cv::Point2f point;
        point.x = object->center_image.at<float>(0, 0); //u
        point.y = object->center_image.at<float>(1, 0); //v

        points.push_back(point);
    }

    for(const mask_rcnn::SemanticObject& object : semantic_objects) {
        bb.push_back(object.bounding_box);
    }

    //make pose out of camera pose
    // vdo_slam_scene->camera_pose = scene->odom_msg().pose.pose;
    // vdo_slam_scene->camera_twist = scene->odom_msg().twist.twist;

    // //we give the msg the same header as the scene, which is the same header time as the original img
    // ros::Time scene_time = scene->get_ros_time();
    // vdo_slam_scene->header.stamp = scene_time;

    //this will be a vector of length scene->get_scene_objects() where the value is an index
    // associating the object semantic object. There will be more semantic objects than secene objects as not everything will be tracked
    std::vector<int> tracked;
    if (!points.empty()) {
        tracked = tracker.assign_tracking_labels(points, bb);
    }
    else {
        tracked.resize(scene_objects.size());
        //fill with -1 such that we dont try and associate
        std::fill(tracked.begin(), tracked.end(), -1);
    }

    // for (SceneObjectPtr& object_ptr : scene->get_scene_objects())
    for(int i = 0; i < scene_objects.size(); i++) {
        int association = tracked[i];
        SceneObjectPtr object_ptr = scene_objects[i];

        //check association was valid
        if (association >= 0 || association < semantic_objects.size()) {
            mask_rcnn::SemanticObject semantic_object = semantic_objects[association];

            // vdo_scene_object_ptr->label = semantic_object.label;

            // vdo_scene_object_ptr->bounding_box = semantic_object.bounding_box;
            object_ptr->label = semantic_object.label;
            object_ptr->bounding_box = BoundingBox::create<vision_msgs::BoundingBox2D>(semantic_object.bounding_box);

            //we naively add the label to the map. Here is an opportunity to check previous label
            //for the same tracking ID and do some probalisitic matching but for now we wont
            // tracking_class_map.insert({vdo_scene_object_ptr->tracking_id, vdo_scene_object_ptr->label});
            tracking_class_map.insert({object_ptr->tracking_id, object_ptr->label});

        }
        else {
            //try to get the previous label from the tracking ID
            if (tracking_class_map.find(object_ptr->tracking_id) != tracking_class_map.end() ) {
                //found -> assign label to scene object. Note, we will not have bounding box
                object_ptr->label = tracking_class_map[object_ptr->tracking_id];
                object_ptr->bounding_box = BoundingBox();
            } 
            else {
                ROS_WARN_STREAM("Tracking association was invalid: Coud not find association in tracking map or by matching");
            }

        }


    }

}



void RosVdoSlam::vdo_worker() {

    std::unique_ptr<VDO_SLAM::Scene> scene;
    // RosScenePtr ros_scene;
    VdoSlamInputPtr input;
    while (ros::ok() && !vdo_input_queue.isShutdown()) {

        if (!vdo_input_queue.empty()) {

            vdo_input_queue.pop(input);
            ros::Time image_time = input->image_time;
            VDO_SLAM::Time slam_time = VDO_SLAM::Time::create<ros::Time>(image_time);
            ROS_INFO_STREAM(slam_time);
            cv::Mat original_rgb;
            input->raw.copyTo(original_rgb);

            ros::Time my_vdo_start_time = ros::Time::now();
            std::pair<SceneType, std::shared_ptr<Scene>> track_result = slam_system->TrackRGBD(input->raw,input->depth,
                input->flow,
                input->mask,
                slam_time,
                slam_time.to_sec(),
                image_trajectory,global_optim_trigger);

            // cv::imshow("Depth", input->depth);
            
            // ros::Duration vdo_time_diff = ros::Time::now() - my_vdo_start_time;
            // std::cout<< "vdo_slam took: "<< vdo_time_diff.toSec() << std::endl;
            // std::cout<< "vdo_slam framerate: " << 1/vdo_time_diff.toSec() << std::endl;

            // std::unique_ptr<VDO_SLAM::Scene> unique_scene =  

            SceneType scene_type = track_result.first;
            vdo_scene = track_result.second;

            std::vector<mask_rcnn::SemanticObject> semantic_objects = input->semantic_objects;

            // ros_scene = std::make_shared<VDO_SLAM::RosScene>(*scene, input->image_time);
            vdo_scene_vector.push_back(vdo_scene);


            // if (scene_type == SceneType::OPTIMIZED) {
            //     ROS_INFO_STREAM("Reconstructing scene!!!!");
            //     slam_system->construct_scenes(vdo_scene_vector);
            //     realtime_vdo_slam::VdoSlamMapPtr map_ptr = RosScene::make_map(vdo_scene_vector);

            //     if (map_ptr != nullptr) {
            //         map_pub.publish(map_ptr);
            //     }
            // }
            
            merge_scene_semantics(vdo_scene, semantic_objects);
            // ros_scene = std::make_shared<VDO_SLAM::RosScene>(*vdo_scene, input->image_time);
            // realtime_vdo_slam::VdoSlamScenePtr summary_msg = ros_scene->to_msg();
            realtime_vdo_slam::VdoSlamScenePtr summary_msg = vdo_scene->convert<realtime_vdo_slam::VdoSlamScenePtr>();
            if(summary_msg != nullptr) {
                //we send a std::shared ptr as the visualizer is in the same node so we maximise sending speed
                //see ros interprocess comms: http://wiki.ros.org/roscpp/Overview/Publishers%20and%20Subscribers#Intraprocess_Publishing
                scene_pub.publish(summary_msg);

            }
            ros::Duration vdo_time_diff = ros::Time::now() - my_vdo_start_time;
            std::cout<< "vdo_slam took: "<< vdo_time_diff.toSec() << std::endl;
            std::cout<< "vdo_slam framerate: " << 1/vdo_time_diff.toSec() << std::endl;
        }
    }

}