#ifndef VDO_SLAM_TYPES_H
#define VDO_SLAM_TYPES_H

#include <opencv2/opencv.hpp>
#include <vdo_slam_g2o/types/types_seven_dof_expmap.h>
#include <memory>
#include <iostream>

#include <chrono>




namespace VDO_SLAM {

     // Input sensor
    enum eSensor {
        MONOCULAR=0,
        STEREO=1, //confusing between stereo and RGBD, for now use RGBD?
        RGBD=2,
        INVALID=3
    };

    enum SceneType {
        SINGLE = 0,
        OPTIMIZED = 1,
        ERROR = 2
    };

    /**
     * @brief Represents a time object with seconds and nano seconds. Makes it easy to
     * convert between this and other representations (eg ROS)
     * 
     */
    class Time {

        public:
            Time() {
                // std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
                // auto duration = now.time_since_epoch();
                
            }

            Time(uint32_t sec_, uint32_t nsec_)
                :   sec(sec_), nsec(nsec_) {}

            Time(const Time& t)
                :   sec(t.sec), nsec(t.nsec) {}

            uint32_t sec, nsec;

            template<class T>
            inline static Time create(T& t);

            template<class T>
            inline T convert();

            friend std::ostream &operator << (std::ostream& output, const Time& object) {
                output << "sec: " << object.sec << " nsec: " << object.nsec << std::endl;
                return output;
            }

    };

    class BoundingBox {

        public:

            BoundingBox() {}
            BoundingBox(double x_, double y_, double width_, double height_);
            BoundingBox(const BoundingBox& bb) {
                x = bb.x;
                y = bb.y;
                width = bb.width;
                height = bb.height;
            }

            template<class T>
            inline static BoundingBox create(T& t);

            template<class T>
            inline T convert();


            cv::Rect2d to_rect();

            //this should follow the same structure as cv::Rect
            double x;
            double y;
            double width;
            double height;

        protected:
            bool is_init = false;
    };


      /**
     * @brief Defines a base class that contains information to
     * describe a physical object in 3D space. Uses the g2o::SE3Quat object
     * to define translational and rotational vectors. 
     * 
     * Includes conversion functions from cv::Mat's in homogenous matrix form. 
     * 
     */
    struct EuclideanObject {

        //TODO: should be unique ptr's but we make shared so so that we can use the 
        //default copy constructor
        std::shared_ptr<g2o::SE3Quat> pose;
        std::shared_ptr<g2o::SE3Quat> twist;

        /**
         * @brief Translation vector in pose
         * 
         * @return const Eigen::Vector3d& 
         */
        inline const Eigen::Vector3d& poseT() {return pose->translation();}

        /**
         * @brief Quaterion vector in pose
         * 
         * @return const Eigen::Quaterniond& 
         */
        inline const Eigen::Quaterniond& poseR() {return pose->rotation();}
        /**
         * @brief Translation vector in twist
         * 
         * @return const Eigen::Vector3d& 
         */
        inline const Eigen::Vector3d& twistT() {return twist->translation();}
        /**
         * @brief Quaternion vector in twist
         * 
         * @return const Eigen::Quaterniond& 
         */
        inline const Eigen::Quaterniond& twistR() {return twist->rotation();}

        /**
         * @brief Sets the pose object from a cv::Mat. The matrix should be in homogenous 
         * ([R | t]) form. If it is in camera frame form ([x,z,y]) use the 
         * utils::image_to_global_coordinates function to convert to [x,y,z].
         * 
         * @param cv::Mat&  pose 
         */
        void pose_from_homogenous_mat(const cv::Mat& pose);

        /**
         * @brief Sets the pose object from a cv::Mat in column vector form [x,y,z] when no rotation is available.
         * We construct a homogenous matrix with rotation = identity matrix.
         * 
         * @param const cv::Mat&  pose 
         */
        void pose_from_vector(const cv::Mat& vector);
        /**
         * @brief Sets the twist object from a cv::Mat. The matrix should be in homogenous 
         * ([R | t]) form. If it is in camera frame form ([x,z,y]) use the 
         * utils::image_to_global_coordinates function to convert to [x,y,z].
         * 
         * @param const cv::Mat& twist 
         */
        void twist_from_homogenous_mat(const cv::Mat& twist);

        /**
         * @brief Sets the twist object from a cv::Mat in column vector form [x,y,z] when no rotation is available.
         * We construct a homogenous matrix with rotation = identity matrix
         * 
         * @param const cv::Mat&  pose 
         */
        void twist_from_vector(const cv::Mat& vector);

    };

    // std::ostream& operator<<(std::ostream& os, const SceneType& scene);

};



#endif
