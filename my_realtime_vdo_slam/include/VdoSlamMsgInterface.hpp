#ifndef _ROS_VDO_SLAM_MSG_INTERFACE
#define _ROS_VDO_SLAM_MSG_INTERFACE


#include "utils/RosUtils.hpp"

#include <ros/ros.h>

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
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <my_realtime_vdo_slam/VdoSlamScene.h>
#include <my_realtime_vdo_slam/VdoSceneObject.h>
#include <my_realtime_vdo_slam/VdoSlamMap.h>


#include <iostream>
#include <stdio.h>
#include <nav_msgs/Odometry.h>
#include <mutex>

#include <vdo_slam/utils/Types.h>
#include <vdo_slam/Scene.h>


namespace VDO_SLAM {

    template<>
    inline Time Time::create<ros::Time>(ros::Time& t) {
        Time time(t.sec, t.nsec);
        return time;
    }

    template<>
    inline ros::Time Time::convert() {
        ros::Time t;
        t.sec = sec;
        t.nsec = nsec;
        return t;
    }

    template<>
    inline SceneObjectPtr SceneObject::create<my_realtime_vdo_slam::VdoSceneObjectConstPtr>(my_realtime_vdo_slam::VdoSceneObjectConstPtr& _msg) {
        SceneObjectPtr scene_object = std::make_shared<SceneObject>();

        g2o::SE3Quat pose_quat = utils::g2o_converter::from_pose_msg(_msg->pose);
        g2o::SE3Quat twist_quat = utils::g2o_converter::from_twist_msg(_msg->twist);

        scene_object->pose = std::make_shared<g2o::SE3Quat>(pose_quat);
        scene_object->twist = std::make_shared<g2o::SE3Quat>(twist_quat);

        scene_object->semantic_instance_index = _msg->semantic_label;
        //this one should have lavel
        //TODO: some assert if label not default - shoudl set this somehwere?
        scene_object->label = _msg->label;
        scene_object->tracking_id = _msg->tracking_id;
        scene_object->unique_id = _msg->uid;

        vision_msgs::BoundingBox2D bb = _msg->bounding_box;
        scene_object->bounding_box = BoundingBox::create<vision_msgs::BoundingBox2D>(bb);
        return scene_object;
    }

    template<>
    inline SceneObjectPtr SceneObject::create<my_realtime_vdo_slam::VdoSceneObject>(my_realtime_vdo_slam::VdoSceneObject& _msg) {
        SceneObjectPtr scene_object = std::make_shared<SceneObject>();
        g2o::SE3Quat pose_quat = utils::g2o_converter::from_pose_msg(_msg.pose);
        g2o::SE3Quat twist_quat = utils::g2o_converter::from_twist_msg(_msg.twist);

        scene_object->pose = std::make_shared<g2o::SE3Quat>(pose_quat);
        scene_object->twist = std::make_shared<g2o::SE3Quat>(twist_quat);

        scene_object->semantic_instance_index = _msg.semantic_label;
        //this one should have lavel
        //TODO: some assert if label not default - shoudl set this somehwere?
        scene_object->label = _msg.label;
        scene_object->tracking_id = _msg.tracking_id;
        scene_object->unique_id = _msg.uid;

        vision_msgs::BoundingBox2D bb = _msg.bounding_box;
        scene_object->bounding_box = BoundingBox::create<vision_msgs::BoundingBox2D>(bb);
        return scene_object;
    }

    template<>
    inline my_realtime_vdo_slam::VdoSceneObjectPtr SceneObject::convert<my_realtime_vdo_slam::VdoSceneObjectPtr>() {
        my_realtime_vdo_slam::VdoSceneObjectPtr msg(new my_realtime_vdo_slam::VdoSceneObject);
        msg->pose = utils::g2o_converter::to_pose_msg(*pose);
        msg->twist = utils::g2o_converter::to_twist_msg(*twist);

        msg->semantic_label = semantic_instance_index;
        msg->label = label;

        //we should not label here becuase the scene object may not have the correct label
        msg->tracking_id = tracking_id;
        msg->time = scene_time.convert<ros::Time>();
        msg->uid = unique_id;
        msg->bounding_box = bounding_box.convert<vision_msgs::BoundingBox2D>();


        return msg;
        
    }

    template<>
    inline SlamScenePtr Scene::create<my_realtime_vdo_slam::VdoSlamSceneConstPtr>(my_realtime_vdo_slam::VdoSlamSceneConstPtr& _msg) {
        SlamScenePtr slam_scene = std::make_shared<Scene>();

        ros::Time stamp_time = _msg->header.stamp;
        slam_scene->frame_id = _msg->id;
        slam_scene->scene_time = VDO_SLAM::Time::create<ros::Time>(stamp_time);
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(_msg->original_frame, sensor_msgs::image_encodings::RGB8);
        slam_scene->rgb_frame = cv_ptr->image;

        g2o::SE3Quat pose_quat = utils::g2o_converter::from_pose_msg(_msg->camera_pose);
        g2o::SE3Quat twist_quat = utils::g2o_converter::from_twist_msg(_msg->camera_twist);

        slam_scene->pose = std::make_shared<g2o::SE3Quat>(pose_quat);
        slam_scene->twist = std::make_shared<g2o::SE3Quat>(twist_quat);

        // *(slam_scene->pose) = utils::g2o_converter::from_pose_msg(_msg->camera_pose);
        // *(slam_scene->twist) = utils::g2o_converter::from_twist_msg(_msg->camera_twist);
        
        for(const my_realtime_vdo_slam::VdoSceneObject& c_object : _msg->objects) {
            my_realtime_vdo_slam::VdoSceneObject obj = c_object;
            SceneObjectPtr object = SceneObject::create<my_realtime_vdo_slam::VdoSceneObject>(obj);
            slam_scene->add_scene_object(object);

        }

        return slam_scene;
    }

    template<>
    inline my_realtime_vdo_slam::VdoSlamScenePtr Scene::convert<my_realtime_vdo_slam::VdoSlamScenePtr>() {
        my_realtime_vdo_slam::VdoSlamScenePtr msg(new my_realtime_vdo_slam::VdoSlamScene);
        msg->camera_pose = utils::g2o_converter::to_pose_msg(*pose);
        msg->camera_twist = utils::g2o_converter::to_twist_msg(*twist);


        msg->id = frame_id;
        msg->header.stamp = scene_time.convert<ros::Time>();
        for (SceneObjectPtr& object : scene_objects) {
            my_realtime_vdo_slam::VdoSceneObject object_msg = *object->convert<my_realtime_vdo_slam::VdoSceneObjectPtr>();
            msg->objects.push_back(object_msg);

        }
        std_msgs::Header header = std_msgs::Header();
        header.stamp = scene_time.convert<ros::Time>();;
        utils::mat_to_image_msg(msg->original_frame, rgb_frame, sensor_msgs::image_encodings::RGB8, header);


        return msg;

    }

}



#endif