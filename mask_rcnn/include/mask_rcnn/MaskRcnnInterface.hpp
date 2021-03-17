#ifndef _MASKRCNN_ROS_INTERFACE
#define _MASKRCNN_ROS_INTERFACE


    // ROS Headers
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <mask_rcnn/SemanticObject.h>



#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <python_service_starter/ServiceStarterInterface.hpp>


#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <vision_msgs/BoundingBox2D.h>
#include <geometry_msgs/Pose2D.h>

// Standard Library Headers
#include <string>
#include <thread>
#include <vector>
#include <map>

//converts rostime to seconds (double)
#define ROS_TIME_TOSEC(time) time.toSec()
#define ROS_TIME_SEC double



namespace mask_rcnn {

    class MaskRcnnInterface : public ServiceStarterInterface {

        public:
            MaskRcnnInterface(ros::NodeHandle& n);
            ~MaskRcnnInterface() {};


            /**
             * @brief Analysises the image image using Mask Rcnn.
             * 
             * @param current_image RGB uint8 image to analyse
             * @param dst Mono uint8 image where background pixels are set to 0 and sematic labels are provided to each pixel.
             * The semantic label can be found at the index of the labels vector at the using the pixel value as the idnex
             * @param viz RGB uint8 image where the masks have been colourized for easier visualisation.
             * @param semantic_objects list of Semantic Objects found in the image
             * @param image_time ros::Time The time the image was generated. Used to add the semantic objects to a map of frames
             * so must be included if request_semantic_objects is to be used
             * @return true 
             * @return false 
             */
            bool analyse(const cv::Mat& current_image, cv::Mat& dst, cv::Mat& viz, std::vector<mask_rcnn::SemanticObject>& semantic_objects,
                ros::Time image_time = ros::Time());

            /**
             * @brief Analysises the image image using Mask Rcnn.
             * 
             * @param current_image RGB uint8 image to analyse
             * @param dst Mono uint8 image where background pixels are set to 0 and sematic labels are provided to each pixel.
             * The semantic label can be found at the index of the labels vector at the using the pixel value as the idex.
             * @param viz RGB uint8 image where the masks have been colourized for easier visualisation.
             * @return true 
             * @return false 
             */
            bool analyse(const cv::Mat& current_image, cv::Mat& dst, cv::Mat& viz);


            /**
             * @brief begins the python program found in the scripts file.
             * 
             * @param wait_for_services if true the function should use the wait for services function to wait for the 
             * appropiate services to be ready
             * @return true 
             * @return false 
             */
            bool start_service(bool wait_for_services = true) override;

            /**
             * @brief Given a list of index's returns a list of labels associated with each index
             * 
             * @param label_indexs A list of pixel level semantic labels ie 0...N
             * @param labels  A list of semantic labels, eg, "background", "person" corresponding to the pixel label
             * @return true 
             * @return false 
             */
            static bool request_labels(const std::vector<int>& label_indexs, std::vector<std::string>& labels);

            /**
             * @brief Requests a single label from an index
             * 
             * @param index A pixel level semantic label ie 0...N
             * @return std::string& 
             */
            static std::string& request_label(int index);


            /**
             * @brief Set the total list of categories that the MaskRcnn Network has been trained on, including the index values. 
             * 
             * In this case, it was trained on the COCO dataset and the full set of categories can be found here:
             * (https://gist.github.com/AruniRC/7b3dadd004da04c80198557db5da4bda). The key of each key value pair is used
             * as the pixel label on the image, which can then correspond with the true semantic label. The request label function
             * simply looks up this pairing once this function has been set.
             * 
             * @param nh 
             * @param timeout 
             * @return true 
             * @return false 
             */
            static bool set_mask_labels(ros::NodeHandle& nh, ros::Duration timeout = ros::Duration(-1));

            /**
             * @brief Waits for the required service to be up: in this case 
             * 'maskrcnn/analyse_image` and 'maskrcnn/request_label' and 'maskrcnn/request_label_list'
             * 
             * @param timeout 
             * @return true 
             * @return false 
             */
            static bool wait_for_services(ros::Duration timeout = ros::Duration(-1));    

            /**
             * @brief Gets the number categoties used by this classifier. Is set with a call to `set_mask_labels`
             * and returns the length of this vector. Returns -1 if the labels have not been set.
             * 
             * @return int 
             */
            static int categories_size();

        private:

            static bool labels_found;
            static std::vector<std::string> mask_labels; //we will request this at the start of the program from the mask rcnn interface
            static std::string invalid_name;
            static std::string coco_file_name;

            

    };

}; //namespace mask_rcnn


#endif
