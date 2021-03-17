#include <ros/ros.h>
#include <sensor_msgs/Image.h>

void imageLeftCallback(const sensor_msgs::Image::ConstPtr& msg){
    ROS_INFO("Left image received - size: %dx%d", msg->width, msg->height);
}


int main(int argc, char **argv){
    ros::init(argc, argv, "video_getter");
    ros::NodeHandle n;
    

    ros::Subscriber subLeftRectified = n.subscribe("/zed2/zed_node/left/image_rect_color",10,imageLeftCallback);
    
    ros::spin();
    return 0;
}