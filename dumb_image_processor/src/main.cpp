#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <stdio.h>



void imageCallback(const sensor_msgs::Image::ConstPtr& msg){

    // ROS_INFO("Left image received - Size: %dx%d", msg->width, msg->height);
    
    cv_bridge::CvImagePtr cv_ptr;
    try{
        cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8);
    }catch (cv_bridge::Exception& e){
        ROS_ERROR("error %s", e.what());
        return;
    }
    cv::namedWindow("image window", CV_WINDOW_NORMAL);
    cv::resizeWindow("image window", 600,600);
    cv::imshow("image window", cv_ptr->image);
    
    cv::waitKey(3);
    
    
}

int main(int argc, char** argv){
    ros::init(argc, argv, "dumb_image_processor_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/zed2/zed_node/right/image_rect_color", 1, imageCallback);

    ros::spin();    

    return 0;

}

// Using native camera
// cv::VideoCapture cap;
//     cap.open(0,cv::CAP_ANY);
//     if(!cap.isOpened()){
//         std::cerr << "error\n";
//         return -1;
//     }
//     cv::Mat frame;
//     while(true){
//         cap.read(frame);
//         if (frame.empty()) {
//             std::cerr << "ERROR! blank frame grabbed\n";
//             break;
//         }
//         // show live and wait for a key with timeout long enough to show images
//         imshow("Live", frame);
//         if (cv::waitKey(5) >= 0)
//             break;
//     }