#include "scene_graph/SceneGraphBuilder.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "ros_scene_graphs");


    VDO_SLAM::SceneGraphBuilder builder;
    
    builder.load_data();
    builder.construct_graph();
    // ros::spin();
    
}
