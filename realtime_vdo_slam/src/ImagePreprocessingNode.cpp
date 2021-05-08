#include "preprocessing/ImagePreprocessing.hpp"
#include "preprocessing/ImagePreprocessingRGB.hpp"
#include "preprocessing/ImagePreprocessingRGBD.hpp"

#include <ros/ros.h>
#include <opencv2/opencv.hpp>

using namespace VDO_SLAM;

void uhumans_depth_preprocessor(const cv::Mat& src, cv::Mat& dst) {
    cv::bitwise_not(src, dst);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "image_preprocessing");
    ros::NodeHandle n;

    
    // mono_depth_2::MonoDepthInterfacePtr mono_depth_ptr = std::make_shared<mono_depth_2::MonoDepthInterface>(n);
    // mono_depth_ptr->attach_preprocessor(boost::bind(&uhumans_depth_preprocessor, _1, _2));

    // mask_rcnn::MaskRcnnInterfacePtr mask_rcnn_ptr = std::make_shared<mask_rcnn::MaskRcnnInterface>(n);
    // flow_net::FlowNetInterfacePtr sceneflow_ptr = std::make_shared<flow_net::FlowNetInterface>(n);
    preprocessing::BaseProcessingPtr processing_ptr;

    preprocessing::InputType type = preprocessing::BaseProcessing::get_input_type(n);

    if (type == preprocessing::InputType::RGB) {
        processing_ptr = std::make_shared<preprocessing::ImageRGB>(n);
    }
    else if (type == preprocessing::InputType::RGB_DEPTH) {
        processing_ptr = std::make_shared<preprocessing::ImageRgbDepth>(n);
    }
    // else if (type == preprocessing::InputType::RGB_DEPTH_SEG) {
    //     processing_ptr = std::make_shared<preprocessing::ImageRgbDepthSeg>(n, mono_depth_ptr, mask_rcnn_ptr, sceneflow_ptr);
    // }
    // else if (type == preprocessing::InputType::RGB_DEPTH_SEG_FLOW) {
    //     processing_ptr = std::make_shared<preprocessing::ImageAll>(n, mono_depth_ptr, mask_rcnn_ptr, sceneflow_ptr);
    // }

    processing_ptr->start_services();

    ros::spin();
}

