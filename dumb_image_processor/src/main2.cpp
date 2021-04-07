#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "raw image";

class Video_Displayer
{
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber sub;
    image_transport::Publisher pub;

    public:
        // init
        Video_Displayer() : it(nh){ //gives the nodehandle to it (as required by the package)
            sub = it.subscribe("/zed2/zed_node/left/image_rect_color", 1, &Video_Displayer::imgCb, this);
            pub = it.advertise("/raw_image",1);
            cv::namedWindow(OPENCV_WINDOW, CV_WINDOW_NORMAL);
        }
        ~Video_Displayer(){
            cv::destroyWindow(OPENCV_WINDOW);
        }
        
        void imgCb(const sensor_msgs::ImageConstPtr& msg){
            cv_bridge::CvImagePtr cv_ptr;
            namespace enc = sensor_msgs::image_encodings;

            try{
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            }catch (cv_bridge::Exception& e){
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }

            // publish the image
            display_img(cv_ptr->image);
            pub.publish(cv_ptr->toImageMsg());
        }

        void display_img(cv::Mat img){
            cv::imshow(OPENCV_WINDOW, img);
            cv::waitKey(3);
        }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dumb_image_processor_node_2");
    Video_Displayer abc;
    ros::spin();
    return 0;
}