#include "mask_rcnn/MaskRcnnInterface.hpp"

#include <mask_rcnn/MaskRcnnVdoSlam.h>
#include <mask_rcnn/MaskRcnnLabel.h>
#include <mask_rcnn/MaskRcnnLabelList.h>
#include <mask_rcnn/SemanticObject.h>

#include <python_service_starter/StartMaskRcnn.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <nlohmann/json.hpp>
#include <ros/package.h>
#include <ros/ros.h>

#include <iostream>
#include <fstream>

using namespace mask_rcnn;
using json = nlohmann::json;

std::string MaskRcnnInterface::coco_file_name = "";

//TOOD put in utils file for PytonServicesInterface
MaskRcnnInterface::MaskRcnnInterface(ros::NodeHandle& n) :
        ServiceStarterInterface(n)
    {
      
    start_client = nh.serviceClient<python_service_starter::StartMaskRcnn>("start_mask_rcnn");
    nh.getParam("/mask_rcnn_interface/coco_dataset", MaskRcnnInterface::coco_file_name );

    ROS_INFO_STREAM("Dataset file name: " << MaskRcnnInterface::coco_file_name );

    }

bool MaskRcnnInterface::start_service(bool wait_for_services) {

    if (ros::service::exists("maskrcnn/analyse_image", true)) {
        service_started = true;
        
        return true;
    }
    // ROS_INFO("I passed here");
    python_service_starter::StartMaskRcnn srv;
    srv.request.start = true;

    service_started = start_client.call(srv);
    ROS_INFO_STREAM("Start masrk rcnn service returned " << service_started);
    //must be initalised after call
    client = nh.serviceClient<mask_rcnn::MaskRcnnVdoSlam>("maskrcnn/analyse_image");

    if (wait_for_services) {
        service_started = MaskRcnnInterface::wait_for_services();
    }

    return service_started;
    
}

bool MaskRcnnInterface::wait_for_services(ros::Duration timeout) {
    return ros::service::waitForService("maskrcnn/analyse_image", timeout);
}    


bool MaskRcnnInterface::analyse(const cv::Mat& current_image, cv::Mat& dst, cv::Mat& viz,
    std::vector<mask_rcnn::SemanticObject>& semantic_objects, ros::Time image_time) {


    if (!service_started) {
        return false;
    }
    sensor_msgs::ImagePtr current_image_msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", current_image).toImageMsg();

    mask_rcnn::MaskRcnnVdoSlam srv;
    srv.request.input_image = *current_image_msg;

    if(client.call(srv)) {

        if (srv.response.success) {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(srv.response.output_mask, sensor_msgs::image_encodings::MONO8);
            dst = cv_ptr->image;

            cv_ptr = cv_bridge::toCvCopy(srv.response.output_viz, sensor_msgs::image_encodings::RGB8);
            viz = cv_ptr->image;
            semantic_objects.clear();

            for (std::vector<mask_rcnn::SemanticObject>::iterator it = srv.response.semantic_objects.begin(); it != srv.response.semantic_objects.end(); ++it) {
                semantic_objects.push_back(*it);
            }

            return true;

        }
        else {
            ROS_ERROR_STREAM("Mask rcnn service returned failed success");
            return false;
        }
        
    }
    else {
        ROS_ERROR_STREAM("Failed to call Mask rcnn service");
        return false;
    }

}



bool MaskRcnnInterface::analyse(const cv::Mat& current_image, cv::Mat& dst, cv::Mat& viz) {
    std::vector<mask_rcnn::SemanticObject> objects;
    return analyse(current_image, dst, viz, objects);
}


std::string MaskRcnnInterface::invalid_name = "invalid";
bool MaskRcnnInterface::labels_found = false;
std::vector<std::string> MaskRcnnInterface::mask_labels{};


//STATIC FUNCTIONS
std::string& MaskRcnnInterface::request_label(int index) {
    if (!MaskRcnnInterface::labels_found) {
        ROS_ERROR_STREAM("Mask labels have not been set. Use set mask labels to get them from the mask rcnn node.");
        return MaskRcnnInterface::invalid_name;
    }
    if (index > mask_labels.size() -1) {
        return invalid_name;
    }
    return mask_labels[index];
}

bool MaskRcnnInterface::request_labels(const std::vector<int>& label_indexs, std::vector<std::string>& labels) {
    if (!MaskRcnnInterface::labels_found) {
        ROS_ERROR_STREAM("Mask labels have not been set. Use set mask labels to get them from the mask rcnn node.");
        return false;
    }

    labels.resize(0);
    if (mask_labels.size() > 0) {
        for (int label_index : label_indexs) {
            std::string label = mask_labels[label_index];
            labels.push_back(label);
        }
        return true;
    }
    else {
        ROS_WARN_STREAM("No labels in mask labels");
        return false;
    }
}

bool MaskRcnnInterface::set_mask_labels(ros::NodeHandle& nh, ros::Duration timeout) {
    if (MaskRcnnInterface::labels_found) {
        return true;
    }

    std::string labels_print;
    if(ros::service::waitForService("maskrcnn/request_label_list", timeout)) {
        ROS_INFO_STREAM("Label service [maskrcnn/request_label_list] active");
        ros::ServiceClient mask_rcnn_labels_list = nh.serviceClient<mask_rcnn::MaskRcnnLabelList>("maskrcnn/request_label_list");
        mask_rcnn::MaskRcnnLabelList all_labels;

        if (mask_rcnn_labels_list.call(all_labels)) {
            mask_labels = all_labels.response.labels;
            for (std::string& label : mask_labels) {
                labels_print += label += ", ";
            }
            ROS_DEBUG_STREAM("Setting mask labels: " << labels_print);
            MaskRcnnInterface::labels_found = true;
            return true;
        }
    }
    else {
         //try to load from source in config folder
        std::string path = ros::package::getPath("realtime_vdo_slam");
        std::string ms_coco_classnames_file = path + "/config/" + coco_file_name;
        ROS_INFO_STREAM("Trying to load dataset class names from: " << ms_coco_classnames_file);
        json json_file;

        std::ifstream file;
        file.exceptions (std::ifstream::badbit);

        try {
            file.open(ms_coco_classnames_file);
            file >> json_file;
        }
        catch (const std::ifstream::failure& e) {
            ROS_WARN_STREAM("Could not load dataset class file.");
            return false;
        }


    }
    return false;

}

int MaskRcnnInterface::categories_size() {
    if (!MaskRcnnInterface::labels_found) {
        return -1;
    }
    return MaskRcnnInterface::mask_labels.size();
}