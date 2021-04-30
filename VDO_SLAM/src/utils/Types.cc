#include "vdo_slam/utils/Types.h"
#include "vdo_slam/utils/VdoUtils.h"
#include "vdo_slam/Macros.h"
#include "vdo_slam/Converter.h"

#include <opencv2/opencv.hpp>

using namespace VDO_SLAM;


BoundingBox::BoundingBox(double x_, double y_, double width_, double height_)
    :   x(x_), y(y_), width(width_), height(height_) {}

cv::Rect2d BoundingBox::to_rect() {
    return cv::Rect2d(x, y, width, height);
}

void VDO_SLAM::EuclideanObject::pose_from_homogenous_mat(const cv::Mat& pose_) {
    assert(pose_.rows == pose_.cols == 4 && "Pose matrix should be in homogenous form");

    g2o::SE3Quat quat = VDO_SLAM::Converter::toSE3Quat(pose_);
    std::shared_ptr<g2o::SE3Quat> quat_ptr = std::make_shared<g2o::SE3Quat>(quat);
    pose = quat_ptr;
}

void VDO_SLAM::EuclideanObject::pose_from_vector(const cv::Mat& vector) {
    if(!utils::is_column_vector(vector)) {
        VDO_ERROR_MSG("Translation vector is not in column vector form [3x1]");
    }
    cv::Mat pose_hom = utils::homogenous_identity();
    pose_hom.at<float>(0, 3) = vector.at<float>(0, 0);
    pose_hom.at<float>(1, 3) = vector.at<float>(2, 0);
    pose_hom.at<float>(2, 3) = vector.at<float>(1, 0);
    pose_from_homogenous_mat(pose_hom);
}

void VDO_SLAM::EuclideanObject::twist_from_homogenous_mat(const cv::Mat& twist_) {
    if(!utils::is_homogenous_matrix(twist_)) {
        VDO_ERROR_MSG("Twist matrix is not in homogenous form: " << twist_);
    }
    g2o::SE3Quat quat = VDO_SLAM::Converter::toSE3Quat(twist_);
    std::shared_ptr<g2o::SE3Quat> quat_ptr = std::make_shared<g2o::SE3Quat>(quat);
    twist = quat_ptr;

}

void VDO_SLAM::EuclideanObject::twist_from_vector(const cv::Mat& vector) {
    if(!utils::is_column_vector(vector)) {
        VDO_ERROR_MSG("Translation vector is not in column vector form [3x1]");
    }    
    cv::Mat twist_hom = utils::homogenous_identity();
    twist_hom.at<float>(0, 3) = vector.at<float>(0, 0);
    twist_hom.at<float>(1, 3) = vector.at<float>(2, 0);
    twist_hom.at<float>(2, 3) = vector.at<float>(1, 0);
    twist_from_homogenous_mat(twist_hom);

}
