#include "scene_graph/SceneGraphBuilder.hpp"
#include "visualizer/RosVisualizer.hpp"
#include "data_provider/RosBagDataProvider.hpp"

#include <functional>
#include <memory>


using namespace VDO_SLAM;
// using std::placeholders::_1;

SceneGraphBuilder::SceneGraphBuilder()
   : nh("vdo_slam_scene_graph")
{
    ROS_INFO_STREAM("Using Ros Bag data provider");
    data_provider = std::make_shared<RosBagDataProvider>();
    data_provider->connect_slam_scene_callback(boost::bind(&SceneGraphBuilder::slam_scene_callback, this, _1));
    data_provider->connect_camera_info_callback(boost::bind(&SceneGraphBuilder::camera_info_callback, this, _1));
    graph = std::make_shared<SceneGraph>();

    ros_viz = std::make_shared<VDO_SLAM::RosVisualizer>();
    ros_viz->connect_handler(ros_viz_handler);
    scene_pub = RosVisualizer::create_viz_pub(nh);
}

void SceneGraphBuilder::load_data() {
    data_provider->spin();
}

void SceneGraphBuilder::slam_scene_callback(const realtime_vdo_slam::VdoSlamSceneConstPtr& scene) {
    slam_scenes.push_back(scene);    

}

void SceneGraphBuilder::camera_info_callback(const sensor_msgs::CameraInfoConstPtr& camera_info_msg) {
    boost::shared_ptr<const sensor_msgs::CameraInfo> msg(boost::const_pointer_cast<sensor_msgs::CameraInfo>(camera_info_msg));
    camera_info = std::make_shared<CameraInformation>(msg);
}

void SceneGraphBuilder::construct_graph() {
    ROS_INFO_STREAM("Constructing graph with " << slam_scenes.size() << " slam messages");
    for (realtime_vdo_slam::VdoSlamSceneConstPtr& slam : slam_scenes) {
        boost::shared_ptr<realtime_vdo_slam::VdoSlamScene> slam_scene_ptr(boost::const_pointer_cast<realtime_vdo_slam::VdoSlamScene>(slam));

        graph->add_dynamic_object(slam_scene_ptr);
    }

    std::map<TrackingId, CurveParamPair> optimized_map;
    graph->optimize_object_poses(optimized_map);
    // graph->show_optimized_poses(map, 4);
    std::vector<realtime_vdo_slam::VdoSlamScenePtr> new_slam_scene = graph->reconstruct_slam_scene(optimized_map);
    for(auto& slam_scene : new_slam_scene) {
        scene_pub.publish(slam_scene);
        ros::spinOnce();
        //need to sleep otherwise the viz will miss all messages
        ros::Duration(0.01).sleep();
    }

}