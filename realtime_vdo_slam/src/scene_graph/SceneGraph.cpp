#include "scene_graph/SceneGraph.hpp"
#include <Eigen/Core>
#include <math.h>


#include "scene_graph/SceneGraphOptimizer.hpp"
#include <algorithm>  

#include <opencv2/opencv.hpp>

#include <ros/ros.h>

SceneGraph::SceneGraph() {
    ROS_INFO_STREAM("using Cauchy loss");
    loss = minisam::CauchyLoss::Cauchy(1.0);
}


void SceneGraph::add_dynamic_object(realtime_vdo_slam::VdoSlamScenePtr& scene) {
    slam_scenes.push_back(scene);

    for(realtime_vdo_slam::VdoSceneObject& object : scene->objects) {
        //TODO: currently not adding slam scene association to SlamObjectAssociation
        //first time we see this dynamic object
        if (dyn_object_map.find(object.tracking_id) == dyn_object_map.end() ) {
            SlamObjectAssociationVector vector;

            SlamObjectAssociation object_association;
            object_association.slam_scene_ptr = scene;
            object_association.object = object;
            object_association.time = object.time;

            vector.push_back(object_association);
            dyn_object_map.insert(std::make_pair(object.tracking_id, vector));
        }
        else {
            SlamObjectAssociation object_association;
            object_association.object = object;
            object_association.time = object.time;
            object_association.slam_scene_ptr = scene;
            dyn_object_map[object.tracking_id].push_back(object_association);
        }
    }
}

void SceneGraph::optimize_object_poses(std::map<TrackingId, CurveParamPair>& optimized_map) {
    // std::map<TrackingId, CurveParamPair> optimized_map;

    //get data for each object
    DynObjectMap::iterator it;
    for (it = dyn_object_map.begin(); it != dyn_object_map.end(); it++) {
        ROS_INFO_STREAM("Collecting data for track id: " << it->first);
        minisam::FactorGraph factor_graph;
    
        
        for (SlamObjectAssociation& object_association : it->second) {
            factor_graph.add(ExpCurveFittingFactor(minisam::key('p', 0), Eigen::Vector2d(object_association.object.pose.position.y,
                                                object_association.object.pose.position.x), loss));

        }

        ROS_INFO_STREAM("Optimizing for " << it->second.size() << " data points");
        minisam::Variables init_values;

        init_values.add(minisam::key('p', 0), Eigen::Vector2d(0, 0));
        ROS_INFO_STREAM("initial curve parameters :"  << init_values.at<Eigen::Vector2d>(minisam::key('p', 0)));

        // optimize!
        minisam::LevenbergMarquardtOptimizerParams opt_param;
        opt_param.verbosity_level = minisam::NonlinearOptimizerVerbosityLevel::ITERATION;
        opt_param.lambda_max = 2e10; //idk what this should be
        minisam::LevenbergMarquardtOptimizer opt(opt_param);

        minisam::Variables values;
        auto status = opt.optimize(factor_graph, init_values, values);
        if(status == minisam::NonlinearOptimizationStatus::SUCCESS) {
            //for now just add if success
            ROS_INFO_STREAM("success");
            Eigen::Vector2d result = values.at<Eigen::Vector2d>(minisam::key('p', 0));
            CurveParamPair pair = std::make_pair(result[0], result[1]);

            // pair.first = result[0]; //m
            // pair.second = result[1]; //c
            optimized_map.insert(std::make_pair(it->first, pair));
            // ROS_INFO_STREAM("opitmized curve parameters :" << values.at<Eigen::Vector2d>(minisam::key('p', 0)));
            ROS_INFO_STREAM("opitmized curve parameters :" << optimized_map[it->first].first << " " <<optimized_map[it->first].second);

        }
        else {
            ROS_INFO_STREAM("fail");
        }

        
    }

}

std::vector<realtime_vdo_slam::VdoSlamScenePtr>& SceneGraph::reconstruct_slam_scene(std::map<TrackingId, CurveParamPair>& optimized_poses) {
    ROS_INFO_STREAM(slam_scenes.size());
    for(realtime_vdo_slam::VdoSlamScenePtr& scene_ptr : slam_scenes) {

        for(realtime_vdo_slam::VdoSceneObject& scene_object : scene_ptr->objects) {

            if(optimized_poses.find(scene_object.tracking_id) != optimized_poses.end()) {
                int track_id = scene_object.tracking_id;
                CurveParamPair& curve_params = optimized_poses[track_id];
                double m = curve_params.first;
                double c = curve_params.second;
                // double smooth_y = exp(m * scene_object.pose.position.x + c);
                double smooth_x = exp(m * scene_object.pose.position.y + c);
                scene_object.pose.position.x = smooth_x;
                // scene_object.pose.position.y = smooth_y;

            }
        }

    }
    return slam_scenes;




}


void SceneGraph::show_optimized_poses(std::map<TrackingId, CurveParamPair>& optimized_poses, int random_samples) {
    //generate n random samples
    //no good way to do this so we make a list of size K (number of total tracking IDs), randomly shuffle and take the first n
    ROS_INFO_STREAM("Randomly sampling: " << random_samples);
    int size = dyn_object_map.size();
    ROS_INFO_STREAM(size);
    std::vector<int> samples(size);
    for (int i = 0; i < size; i++) {
        samples[i] = i;
    }

    std::random_shuffle(samples.begin(), samples.end());
    if (random_samples == -1) {
        random_samples = size;
    }

    cv::Mat display = cv::Mat::zeros(800, 800, CV_8UC3);

    //currently same code as in RosVisualizer -> coudl put into config file so all nodes can see
    const int x_offset = 150;
    const int y_offset = 150;
    const int scale = 6;

    //i will be the tracking ID
    for(int i = 0; i < random_samples; i++) {
        int track = samples[i];
        ROS_INFO_STREAM(track);

        SlamObjectAssociationVector& object_vector = dyn_object_map[track];
        CurveParamPair& curve_params = optimized_poses[track];
        double m = curve_params.first;
        double c = curve_params.second;

        for (SlamObjectAssociation& object_association : object_vector) {
            ROS_INFO_STREAM_ONCE("Plotting original path for track " << track << " and class " << object_association.object.label);
            double x = object_association.object.pose.position.x;
            double y = object_association.object.pose.position.y;

            //plot original path
            int x_display =  static_cast<int>(x*scale) + x_offset;
            int y_display =  static_cast<int>(y*scale) + y_offset;
            cv::circle(display, cv::Point(x_display, y_display), 2, CV_RGB(128, 0, 128), 5); // orange

            //construct smooth path using y=exp(mx+c)
            double smooth_y = exp(m * x + c);
            y_display =  static_cast<int>(smooth_y*scale) + y_offset;
            cv::circle(display, cv::Point(x_display, y_display), 2, CV_RGB(0,0,255), 5); // green

            cv::imshow("Optimized Poses", display);
            cv::waitKey(1);
        }

    }



    cv::imshow("Optimized Poses", display);
    cv::waitKey();


}