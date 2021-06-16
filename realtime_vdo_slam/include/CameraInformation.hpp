#ifndef _ROS_VDO_CAMERA_INFORMATION
#define _ROS_VDO_CAMERA_INFORMATION

#include <sensor_msgs/CameraInfo.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>


#include <string>
#include <memory>


namespace VDO_SLAM {

    enum class DistortionModel {
        NONE,
        RADTAN,
        RAD_POLY,
        EQUIDISTANT,
        PLUMB_BOB,
        INVALID
    };

    /**
     * @brief Converts the encoding of an image msg. Any string encoding may be used but sensor_msgs::encodings is prefered.
     * 
     * @param msg The original image msg
     * @param encoding The encoding to re-encode the image to
     * @return cv_bridge::CvImagePtr An image ptr
     */
    cv_bridge::CvImagePtr convert_img_msg(const sensor_msgs::ImageConstPtr& msg, const std::string& encoding);


    //wrapper for camera information
    struct CameraInformation {

        std::string topic;
        cv::Mat camera_matrix; //K
        cv::Mat dist_coeffs; //D

        //This will be P (see OpenCV docs for estimateNewCameraMatrixForUndistortRectify)
        cv::Mat map1, map2;
        cv::Mat modified_camera_matrix; //P or modified K
        sensor_msgs::CameraInfo camera_info_msg; //original msg
        DistortionModel distortion_model; 

        CameraInformation() {}
        CameraInformation(sensor_msgs::CameraInfoConstPtr& info_msg_ptr);
        void apply_undistortion(const cv::Mat& src, cv::Mat& dst);



    };

    typedef std::shared_ptr<CameraInformation> CameraInformationPtr;
    typedef std::unique_ptr<CameraInformation> CameraInformationUniquePtr;

}


#endif
