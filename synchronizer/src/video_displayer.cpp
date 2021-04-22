#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


class Video_Displayer
{
    private:
        ros::NodeHandle nh;
        image_transport::ImageTransport it;
        image_transport::Subscriber sub;
        image_transport::Publisher pub;
        std::string _cvWindow_name;
        std::string _subTopicName;

    public:
        // init
        Video_Displayer(std::string subTopicName, std::string cvWindowName) : it(nh){ //gives the nodehandle to it (as required by the package)
            
            _subTopicName = subTopicName;
            _cvWindow_name = cvWindowName;

            sub = it.subscribe(_subTopicName, 1, &Video_Displayer::imgCb, this);
            // pub = it.advertise("/raw_image",1);

            cv::namedWindow(_cvWindow_name, CV_WINDOW_NORMAL);
        }
        ~Video_Displayer(){
            cv::destroyWindow(_cvWindow_name);
        }
        
        void imgCb(const sensor_msgs::ImageConstPtr& msg){
            cv_bridge::CvImagePtr cv_ptr;
            // namespace enc = sensor_msgs::image_encodings;

            try{
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            }catch (cv_bridge::Exception& e){
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }

            display_img(cv_ptr->image);
            // publish the image
            // pub.publish(cv_ptr->toImageMsg());
        }

        void display_img(cv::Mat img){
            cv::imshow(_cvWindow_name, img);
            cv::waitKey(3);
        }
        
};
