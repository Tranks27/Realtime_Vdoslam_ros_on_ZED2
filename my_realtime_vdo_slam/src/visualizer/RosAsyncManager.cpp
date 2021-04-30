#include "visualizer/RosAsyncManager.hpp"

using namespace VDO_SLAM;

RosAsyncManager::RosAsyncManager(RosCallbackQueuePtr& _callback_queue)
{
    callback_queue_ptr = _callback_queue;
}



void RosAsyncManager::connect_callback(const ros::SingleSubscriberPublisher& pub) {
    std::string topic = pub.getTopic();
    ROS_INFO_STREAM(topic << " connected");
}
void RosAsyncManager::disconnect_callback(const ros::SingleSubscriberPublisher& pub) {
    std::string topic = pub.getTopic();
    ROS_INFO_STREAM(topic << " unsubscribed");
}

void RosAsyncManager::connect_callback_image(const image_transport::SingleSubscriberPublisher& pub) {
    std::string topic = pub.getTopic();
    ROS_INFO_STREAM(topic << " connected");
}
void RosAsyncManager::disconnect_callback_image(const image_transport::SingleSubscriberPublisher& pub) {
    std::string topic = pub.getTopic();
    ROS_INFO_STREAM(topic << " unsubscribed");
}