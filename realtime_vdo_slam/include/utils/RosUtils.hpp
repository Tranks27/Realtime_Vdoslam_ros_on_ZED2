#ifndef _VDO_ROS_UTILS
#define _VDO_ROS_UTILS

#include <string>

#include <geometry_msgs/Transform.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/TransformStamped.h>
#include <vision_msgs/BoundingBox2D.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>

#include <opencv2/opencv.hpp>

#include <vdo_slam_g2o/types/types_seven_dof_expmap.h>
#include <vdo_slam/utils/Types.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>


namespace VDO_SLAM {

    template<>
    inline BoundingBox BoundingBox::create<vision_msgs::BoundingBox2D>(vision_msgs::BoundingBox2D& t) {
        return BoundingBox(t.center.x, t.center.y, t.size_x, t.size_y);
    }

    template<>
    inline vision_msgs::BoundingBox2D BoundingBox::convert<vision_msgs::BoundingBox2D>() {
        vision_msgs::BoundingBox2D bb;
        bb.center.x = x;
        bb.center.y = y;
        bb.size_x = width;
        bb.size_y = height;
        return bb;
    }


    namespace utils {

        void mat_to_image_msg(sensor_msgs::Image& img_msg, const cv::Mat& img, const std::string& encoding, const std_msgs::Header& header = std_msgs::Header());
        void image_msg_to_mat(cv::Mat& img, const sensor_msgs::Image& image_msg, const std::string& encoding);

        /**
         * @brief Converts an odom to a transform with header details and 
         * updates the tf tree
         * 
         * @param odom const nav_msgs::Odometry&
         * @param parent_frame_id const std::string& 
         * @param child_frame_id const std::string& 
         * @param time const ros::Time& defaults to ros::now()
         */
        void publish_static_tf(const nav_msgs::Odometry& odom,
                                const std::string& parent_frame_id,
                                const std::string& child_frame_id, const ros::Time& time = ros::Time::now());

        void odom_to_tf(const nav_msgs::Odometry& odom, geometry_msgs::Transform* transform);

        /**
         * @brief Takes a pose and twist pair and converts them to a Odometry msg.
         * Header frame information is not added.
         * 
         * @param pose const geometry_msgs::Pose
         * @param twist const geometry_msgs::Twist
         * @param time const ros::Time
         * @param odom nav_msgs::Odometry
         */
        void geometry_to_odom(const geometry_msgs::Pose& pose, const geometry_msgs::Twist& twist, const ros::Time& time, nav_msgs::Odometry& odom);


        /**
         * @brief Takes a pose and twist pair and converts them to a Odometry msg.
         * Header frame information will be added.
         * 
         * @param pose const geometry_msgs::Pose
         * @param twist  const geometry_msgs::Twist
         * @param time const ros::Time
         * @param frame_id const std::string 
         * @param child_frame_id const std::string 
         * @param odom nav_msgs::Odometry
         */
        void geometry_to_odom(const geometry_msgs::Pose& pose, const geometry_msgs::Twist& twist, const ros::Time& time, 
                                        const std::string& frame_id, const std::string& child_frame_id, nav_msgs::Odometry& odom);

        /**
         * @brief Apply transform between two objects. The objects should contain a header but will only be transformed in translation space (and not orientation)
         * tf::Listener::transformPoint() can be used if the object T can be converted into a geometry_msgs::PointStamped and then converted back.
         * 
         * @tparam T 
         * @param from_frame 
         * @param target_frame 
         * @param object 
         * @param transformed_object 
         * @param time 
         */
        template<typename T>
        void apply_transform(const std::string& from_frame, const std::string& target_frame, const T& object, T* transformed_object, ros::Time& time = ros::Time());



        /**
         * Determine the correct UTM letter designator for the
         * given latitude
         *
         * @returns 'Z' if latitude is outside the UTM limits of 84N to 80S
         *
         * Taken from: https://github.com/bsb808/geonav_transform/blob/master/include/geonav_transform/navsat_conversions.h
         */
        char UTMLetterDesignator(double Lat);

        /**
         * Convert lat/long to UTM coords.  Equations from USGS Bulletin 1532
         *
         * East Longitudes are positive, West longitudes are negative.
         * North latitudes are positive, South latitudes are negative
         * Lat and Long are in fractional degrees
         *
         * Taken from: https://github.com/bsb808/geonav_transform/blob/master/include/geonav_transform/navsat_conversions.h
         */
        void LLtoUTM(const double Lat, const double Long,
                           double &UTMNorthing, double &UTMEasting,
                           std::string &UTMZone);

        namespace geometry_converter {

            /**
             * @brief Converts a Quaternion msg into a Eigen Quaternion. Useful for converting between
             * ROS pose objects and g2o classes which use Eigen as their base.
             * 
             * @param const geometry_msgs::Quaternion& quat 
             * @return Eigen::Quaterniond 
             */
            Eigen::Quaterniond quat_from_orientation(const geometry_msgs::Quaternion& quat);
            /**
             * @brief Converts a Point msg into a Eigen Vector3d. Useful for converting between
             * ROS pose objects and g2o classes which use Eigen as their base.
             * 
             * @param const const geometry_msgs::Point& point 
             * @return Eigen::Vector3
             */
            Eigen::Vector3d vector_from_translation(const geometry_msgs::Point& point);
            /**
             * @brief Overload for twist msg (which uses geometry msgs Vector 3 and not point)
             * 
             * @param point geometry_msgs::Vector3&
             * @return Eigen::Vector3d 
             */
            Eigen::Vector3d vector_from_translation(const geometry_msgs::Vector3& point);

            /**
             * @brief Converts an Eigen Quaternion into a quaternion ROS msg. Useful for converting back to
             * a g2o object from a pose object.
             * 
             * @param orientation const Eigen::Quaterniond
             * @return geometry_msgs::Quaternion 
             */
            geometry_msgs::Quaternion orienation_from_quat(const Eigen::Quaterniond& orientation);
   
            /**
             * @brief Converts an Eigen Vector3d into a point ROS msg. Useful for converting back to
             * a g2o object from a pose object.
             * 
             * @param vector const Eigen::Vector3d
             * @return geometry_msgs::Point 
             */
            geometry_msgs::Point translation_from_vector(const Eigen::Vector3d& vector);

        }

        namespace g2o_converter {

            g2o::SE3Quat from_pose_msg(const geometry_msgs::Pose& pose_msg);
            geometry_msgs::Pose to_pose_msg(const g2o::SE3Quat& pose);


            //for twist message we only care about linear component becuase thats all we get from
            //VDO slam so only translation will be filled out
            g2o::SE3Quat from_twist_msg(const geometry_msgs::Twist& twist_msg);
            geometry_msgs::Twist to_twist_msg(const g2o::SE3Quat& twist);

        }
    }

}


template<>
inline void VDO_SLAM::utils::apply_transform<nav_msgs::Odometry>(const std::string& from_frame, const std::string& target_frame, const nav_msgs::Odometry& object, nav_msgs::Odometry* transformed_object, ros::Time& time) {
    tf::TransformListener listener(ros::Duration(2));

    geometry_msgs::PointStamped object_point;
    object_point.header.frame_id = from_frame;

    object_point.header.stamp = time;

    //just an arbitrary point in space
    object_point.point.x = object.pose.pose.position.x;
    object_point.point.y = object.pose.pose.position.y;
    object_point.point.z = object.pose.pose.position.z;

    try {
        geometry_msgs::PointStamped transformed_point;

        listener.waitForTransform(target_frame, from_frame,
                              ros::Time::now(), ros::Duration(3.0));
        listener.transformPoint(target_frame, object_point, transformed_point);

        transformed_object->header.frame_id = target_frame;
        transformed_object->pose.pose.position.x = transformed_point.point.x;
        transformed_object->pose.pose.position.y = transformed_point.point.y;
        transformed_object->pose.pose.position.z = transformed_point.point.z;

    }
    catch(tf2::TransformException& ex) {
        ROS_ERROR_STREAM("Received an exception trying to transform a point from" << from_frame << " to " << target_frame <<": " <<  std::string(ex.what()));
    }
}

template<>
inline void VDO_SLAM::utils::apply_transform<cv::Point3d>(const std::string& from_frame, const std::string& target_frame, const cv::Point3d& object, cv::Point3d* transformed_object, ros::Time& time) {
    tf::TransformListener listener(ros::Duration(2));

    geometry_msgs::PointStamped object_point;

    object_point.header.frame_id = from_frame;
    //just an arbitrary point in space
    object_point.point.x = object.x;
    object_point.point.y = object.y;
    object_point.point.z = object.z;

    try {
        geometry_msgs::PointStamped transformed_point;

        listener.waitForTransform(target_frame, from_frame,
                              ros::Time::now(), ros::Duration(3.0));
        listener.transformPoint(target_frame, object_point, transformed_point);

        transformed_object->x = transformed_point.point.x;
        transformed_object->y = transformed_point.point.y;
        transformed_object->z = transformed_point.point.z;

    }
    catch(tf2::TransformException& ex) {
        ROS_ERROR_STREAM("Received an exception trying to transform a point from " << from_frame << " to " << target_frame << ": " <<  std::string(ex.what()));
    }
}

#endif
