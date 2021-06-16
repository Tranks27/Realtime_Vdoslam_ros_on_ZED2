#include "CameraInformation.hpp"
#include <sensor_msgs/CameraInfo.h>
#include "utils/RosUtils.hpp"
#include <cv_bridge/cv_bridge.h>


#include <ros/ros.h>
#include <opencv2/opencv.hpp>

namespace enc = sensor_msgs::image_encodings;


int image_encoding_to_opencv(const std::string& encoding) {
    if (encoding == enc::RGB8) {
        return CV_8UC3;
    }
    else if (encoding == enc::MONO8) {
        return CV_8UC1;
    }
    else if (encoding == enc::MONO16) {
        return CV_16UC1;
    }
    else if(encoding == enc::TYPE_32FC1) {
        return CV_32FC1;
    }
    else {
        ROS_ERROR_STREAM("have not coded conversion for " << encoding << " yet");
        return 0;
    }
    
}

cv_bridge::CvImagePtr VDO_SLAM::convert_img_msg(const sensor_msgs::ImageConstPtr& msg, const std::string& encoding) {
    cv::Mat mat;
    sensor_msgs::Image image_msg = *msg;
    VDO_SLAM::utils::image_msg_to_mat(mat, image_msg, msg->encoding);
    int cv_encoding = image_encoding_to_opencv(encoding);
    mat.convertTo(mat, cv_encoding);

    VDO_SLAM::utils::mat_to_image_msg(image_msg, mat, encoding);


    return cv_bridge::toCvCopy(image_msg, encoding);
}

VDO_SLAM::CameraInformation::CameraInformation(sensor_msgs::CameraInfoConstPtr& info_msg_ptr) {
    camera_info_msg = *info_msg_ptr;

    uint32_t image_width = camera_info_msg.width;
    uint32_t image_height = camera_info_msg.height;

    cv::Size image_size = cv::Size(image_width, image_height);

    if (camera_info_msg.distortion_model == "rational_polynomial") {
        camera_matrix = cv::Mat(3, 3, CV_64F, &camera_info_msg.K[0]);
        dist_coeffs = cv::Mat(4, 1, CV_64F, &camera_info_msg.D[0]);

        distortion_model = DistortionModel::RAD_POLY;

    }
    else if (camera_info_msg.distortion_model == "equidistant") {
        camera_matrix = cv::Mat(3, 3, CV_64F, &camera_info_msg.K[0]);
        dist_coeffs = cv::Mat(4, 1, CV_64F, &camera_info_msg.D[0]);

        cv::Mat identity_mat = cv::Mat::eye(3, 3, CV_64F);

        cv::fisheye::estimateNewCameraMatrixForUndistortRectify(camera_matrix, dist_coeffs, image_size,
                                                                identity_mat, modified_camera_matrix);

        cv::fisheye::initUndistortRectifyMap(camera_matrix,
                                            dist_coeffs,
                                            identity_mat,
                                            modified_camera_matrix,
                                            image_size,
                                            CV_16SC2,
                                            map1, map2);
        distortion_model = DistortionModel::EQUIDISTANT;
    }
    else if (camera_info_msg.distortion_model == "plumb_bob"){
        camera_matrix = cv::Mat(3, 3, CV_64F, &camera_info_msg.K[0]);
        dist_coeffs = cv::Mat(4, 1, CV_64F, &camera_info_msg.D[0]);

        distortion_model = DistortionModel::PLUMB_BOB;
    }
    else {
        ROS_WARN_STREAM("Distortion Model [" << camera_info_msg.distortion_model << " not recognised.");
        camera_matrix = cv::Mat(3, 3, CV_64F, &camera_info_msg.K[0]);
        dist_coeffs = cv::Mat(4, 1, CV_64F, &camera_info_msg.D[0]);
        distortion_model = DistortionModel::INVALID;
    }
}

void VDO_SLAM::CameraInformation::apply_undistortion(const cv::Mat& src, cv::Mat& dst) {
    if (distortion_model == DistortionModel::INVALID) {
        ROS_WARN_STREAM("Distortian model not set. Cannot undistort image");
        return;
    }

    if (camera_info_msg.distortion_model == "rational_polynomial") {
        cv::undistort(src, dst, camera_matrix, dist_coeffs);
    }
    else if (camera_info_msg.distortion_model == "equidistant") {
        cv::fisheye::undistortImage(src, dst, camera_matrix, dist_coeffs, modified_camera_matrix);
    }
    else if (camera_info_msg.distortion_model == "plumb_bob") {
        ROS_WARN_STREAM("Plumb bob model undistortion function not set. Don't try to undistort image");
        return;
    }
}
