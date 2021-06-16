#include <ros/ros.h>
#include "tracking/SemanticTracker.hpp"




double get_euclid_distance(const vision_msgs::BoundingBox2D& bb, const cv::Point2f& point) {


    //get centroid -> I decided center x and y would represent starting coordinate of bb and not centroid
    double center_x = bb.size_x/2.0 + bb.center.x;
    double center_y = bb.size_y/2.0 + bb.center.y;

    double u = static_cast<double>(point.x);
    double v = static_cast<double>(point.y);

    double dx = center_x - u;
    double dy = center_y - v;

    return std::sqrt((dx * dx) + (dy * dy));
}

SemanticTracker::SemanticTracker() {}


std::vector<int> SemanticTracker::assign_tracking_labels(const std::vector<cv::Point2f>& vdo_object_centroids, 
                                    const std::vector<vision_msgs::BoundingBox2D>& semantic_object_bb) {

    int object_centroids_length = vdo_object_centroids.size();                                   
    int semantic_object_length = semantic_object_bb.size();

    //do I now want to pop back?
    std::vector<double> v(semantic_object_length);

    //solve for rows -> eg current assignment 
    //rows should be vdo objects and cols should be bounding boxes
    std::vector<std::vector<double>> euclid_cost_matrix(object_centroids_length,v);
    for (int i = 0; i < object_centroids_length; i++) {
        // std::vector<double> cols;
        for (int j = 0; j < semantic_object_length; j++) {
            double cost = get_euclid_distance(semantic_object_bb[j], vdo_object_centroids[i]);
            euclid_cost_matrix[i][j] = cost;
        }

    }

    std::vector<int> assignment;
    //Match the semantic bb with vdo objects using hungarian solver for max IOU values
    //Store the costs in the assignment vector 
    double cost = hungarian_solver.Solve(euclid_cost_matrix, assignment); 

    assert(assignment.size() == object_centroids_length);

    return assignment;



}