#ifndef VDO_SLAM_UTILS_H
#define VDO_SLAM_UTILS_H

#include <opencv2/opencv.hpp>
#include "vdo_slam/Macros.h"

#include "vdo_slam/utils/Types.h"

#include <memory>



namespace VDO_SLAM {

    template <typename T, typename... Args>
    std::unique_ptr<T> make_unique(Args&&... args) {
        return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
    }

    namespace utils {
        /**
         * @brief Converts an integer to its associated eSensor type
         * 
         * MONOCULAR = 0
         * STEREO = 1
         * RGBD = 2
         * INVALID = 3 (this will be returned if sensor is not 0, 1 or 2.)
         * 
         * @param sensor 
         * @return eSensor 
         */
        eSensor param_to_sensor(int sensor);

        /**
         * @brief Constructs a 4x4 matrix in the homogenous form (R|t) where R
         * is the identity matrix and t is a 0 column vector
         * 
         * @return cv::Mat 
         */
        cv::Mat homogenous_identity();

        /**
         * @brief Checks that a matrix is in homogenous form; that is 4x4 with [0 0 0 1]
         * as the bottom row.
         * 
         * @param mat const cv:Mat&
         * @return true 
         * @return false 
         */
        bool is_homogenous_matrix(const cv::Mat& mat);

        /**
         * @brief Checks that a matrix is in column vector form (ie Nx1). Important to 
         * check for indexing a matix. In this case we index [i, 0]
         * 
         * @param mat const cv:Mat&
         * @param rows int Number of rows to check that the matrix is. Defaults to 3
         * as we almost always check in the case of translation matrix.
         * @return true 
         * @return false 
         */
        bool is_column_vector(const cv::Mat& mat, int rows = 3);

         /**
         * @brief Checks that a matrix is in rows vector form (ie 1xN). Important to 
         * check for indexing a matix. In this case we index [0, i]
         * 
         * @param mat const cv:Mat&
         * @param rows int Number of rows to check that the matrix is.  Defaults to 3
         * as we almost always check in the case of translation matrix.
         * @return true 
         * @return false 
         */
        bool is_row_vector(const cv::Mat& mat, int cols = 3);

        /**
         * @brief Converts x,y, z coordinates from camera coordinates to global coordinate system
         * When coordinates are calculated we use the standard image place coordinate system
         * but all homogenous calculations later down the line (and for display) require global
         * coordinates. This is equialent to simply swapping the values of y,z. 
         * 
         * @param image const cv::Mat&. Either a 4x4 homogenous matrix of a 3x1 translation vector.
         * @param dst  cv::Mat&. Should be of the same as the input or an error will be thrown.
         */
        void image_to_global_coordinates(const cv::Mat& image, cv::Mat& dst);


    }  //namespace utils


} //namespace VDO_SLAM

#endif