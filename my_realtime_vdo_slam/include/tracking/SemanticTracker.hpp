#ifndef SEMANTIC_TRACKER_HPP
#define SEMANTIC_TRACKER_HPP

    // ROS Headers
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <mask_rcnn/SemanticObject.h>
#include "tracking/HungarianSolver.hpp"


#include <my_realtime_vdo_slam/VdoSlamScene.h>

#include <geometry_msgs/PoseStamped.h>
#include <vision_msgs/BoundingBox2D.h>
#include <geometry_msgs/Pose2D.h>

// Standard Library Headers
#include <string>
#include <thread>
#include <vector>
#include <map>
#include <deque>




class SemanticTracker {

    public:
        SemanticTracker();
        /**
         * Assign a point to a bounding box. The points will represent centroids of the objects found with the VDO slam 
         * algorithm and be in (u,v) coordinate form. The Bounding Box's will detail the outline of the object as determined
         * by mask rcnn and are part of the SemanticObject msg type. As specified in that message file; 
         * center (x,y) will be starting x, y which is the bottom left corner of the image
         * and size_x and size_y will be with and height of image. 
         * 
         * @param vdo_object_centroids const std::vector<cv::Point2f>&
         * @param semantic_object_bb const std::vector<vision_msgs::BoundingBox2D>& 
         * @return std::vector<int> A vector of assignment of length vdo_object_centroids where each value represents the index o
         * assignment in the semantic_object_bb array.
         */
        std::vector<int> assign_tracking_labels(const std::vector<cv::Point2f>& vdo_object_centroids, 
                                    const std::vector<vision_msgs::BoundingBox2D>& semantic_object_bb);


    private:
        HungarianAlgorithm hungarian_solver;
        
};


#endif

