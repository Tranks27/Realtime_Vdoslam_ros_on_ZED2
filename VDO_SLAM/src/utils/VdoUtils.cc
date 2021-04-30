#include "vdo_slam/utils/VdoUtils.h"

#include <opencv2/opencv.hpp>


namespace VDO_SLAM {
    namespace utils {

        eSensor param_to_sensor(int sensor) {
            if (sensor == 0) {
                VDO_INFO_MSG("Using MONOCULAR sensor.");
                return VDO_SLAM::eSensor::MONOCULAR;
            }
            else if (sensor == 1) {
                VDO_INFO_MSG("Using STEREO sensor.");
                return VDO_SLAM::eSensor::STEREO;
            }
            else if (sensor == 2) {
                VDO_INFO_MSG("Using RGBD sensor.");
                return VDO_SLAM::eSensor::RGBD;
            }
            else {
                VDO_ERROR_MSG("Sensor type " << sensor << " is invalid.");
                return VDO_SLAM::eSensor::INVALID;
            }

        }

        cv::Mat homogenous_identity() {

            cv::Mat identity = (cv::Mat_<float>(4,4) << 1, 0, 0, 0,
                                                        0, 1, 0, 0,
                                                        0, 0, 1, 0,
                                                        0, 0, 0, 1);
            return identity;
        }

        bool is_homogenous_matrix(const cv::Mat& mat) {
            if(mat.rows != 4 || mat.cols != 4) {
                VDO_DEBUG_MSG("Mat had size: " << mat.size());
                return false;
            }

            cv::Mat bottom_row = mat.colRange(0, 4).row(3);
            cv::Mat gt_bottom_row = (cv::Mat_<float>(1,4) << 0, 0, 0, 1);
            cv::Mat diff = gt_bottom_row != bottom_row;
            // Equal if no elements disagree
            bool eq = cv::countNonZero(diff) == 0;


            if (!eq) {
                VDO_DEBUG_MSG("homo mat: " << mat);
                return false;
            }
            return true;
        }


        bool is_column_vector(const cv::Mat& mat, int rows) {
            if (mat.rows == rows && mat.cols == 1) {
                return true;
            }
            return false;
        }


        bool is_row_vector(const cv::Mat& mat, int cols) {
            if (mat.cols == cols && mat.rows == 1) {
                return true;
            }
            return false;
        }

        void image_to_global_coordinates(const cv::Mat& image, cv::Mat& dst) {
            image.copyTo(dst);

            if(utils::is_homogenous_matrix(image)) {
                float temp_y = image.at<float>(2,3);
                float temp_z = image.at<float>(1,3);
                //make into normal homogenous matrix form
                dst.at<float>(1,3) = temp_y;
                dst.at<float>(2,3) = temp_z;
            }
            else if(utils::is_column_vector(image)) {
                float temp_y = image.at<float>(2,0);
                float temp_z = image.at<float>(1,0);
                //make into normal homogenous matrix form
                dst.at<float>(1,0) = temp_y;
                dst.at<float>(2,0) = temp_z;
            }
            else if(utils::is_row_vector(image)) {
                float temp_y = image.at<float>(0,2);
                float temp_z = image.at<float>(0,1);
                //make into normal homogenous matrix form
                dst.at<float>(0,1) = temp_y;
                dst.at<float>(0,2) = temp_z;

                //make into col vector
                dst.reshape(3, 1);
            }
            else {
                VDO_ERROR_MSG("Unable to convert matrix to global coordinates: Not homogenous or col vector");
            }
        }

    }
}

