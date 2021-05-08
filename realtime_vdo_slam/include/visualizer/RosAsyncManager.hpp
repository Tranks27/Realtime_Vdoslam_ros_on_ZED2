#ifndef _ROS_VDO_ASYNC_PUBLISHER
#define _ROS_VDO_ASYNC_PUBLISHER



#include <mutex>
#include <future>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/NavSatFix.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <image_transport/image_transport.h>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <nav_msgs/Odometry.h>
#include <image_transport/image_transport.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <opencv2/opencv.hpp>
#include <realtime_vdo_slam/VdoSlamScene.h>

#include <vdo_slam/utils/ThreadedQueue.hpp>

typedef std::string Topic;
typedef std::shared_ptr<ros::CallbackQueue> RosCallbackQueuePtr;

namespace VDO_SLAM {

    /**
     * @brief Managers a specific ros::CallbackQueue and adds publishers that will use this queue to be emptied asynchronously
     * when the spinner is started. A single callback queue is given to the class and then the publisher is created. This takes the place
     * of usual ros::Publisher pub = nh.advertise<Type>(topic, queue_size) syntax.
     * 
     * The create function is used instead and works for both image transport and default (ros::NodeHandle) nodes, as long as the correct MsgType
     * and handler is provided to the function. 
     * 
     */
    class RosAsyncManager {


        public:
            /**
             * @brief Construct a new RosAsyncManager object. This objects take a pointer to a specific callback queue and all publishers
             * will add their messages to this queue (rather than the global callback queue). If this callback queue is attached to a sync spinner
             * any call to publish will result in the messages being published asynchronously.
             * 
             * Note: The callback queue here is not set up to any spinner. This must be done after the publishers are set up. An example setup could be:
             * 
             * RosCallbackQueuePtr callback;
             * RosAsyncPublisher async_pubs(callback);
             * 
             * ros::Publisher pub;
             * ros::NodeHandle nh;
             * 
             * async.create<std_msgs::String>("/chatter", pub, nh);
             * 
             * ros::AsyncSpinner spinner(2, callback.get());
             * 
             * This will set the publisher object to advertise a String message on the topic /chatter, using the queue 'callback'. Once set up, 
             * the callback queue is attached an asynchronous spinner. 
             * 
             * @param _callback_queue RosCallbackQueuePtr&
             */
            RosAsyncManager(RosCallbackQueuePtr& _callback_queue);
            ~RosAsyncManager() {};

            /**
             * @brief Sets up a ros::Publisher to to publish to an synchronous ros callback queue. This takes the place of
             * pub = nh.advertise<Type>(...).
             * 
             * @tparam MsgType The ros msg type this publisher will transport. In this case, MsgType should be anything other than
             * sensor_msgs::Image, as we have a separate function for that.
             * @param topic const std::string& The name of the topic to advertise.
             * @param pub ros::Publisher&  The publisher to connect.
             * @param node_handle ros::NodeHandle& The node handler used to initalise the publisher. 
             * @param queue_size int The queue size to use. Defaults to 20.
             */
            template <class MsgType>
            void create(const Topic& topic, ros::Publisher& pub, ros::NodeHandle& node_handle, int queue_size = 20);

            /**
             * @brief Sets up a image_transport::Publisher to to publish to an synchronous ros callback queue. This takes the place of
             * pub = nh.advertise(...).
             * 
             * @tparam MsgType Must be sensor_msgs::Image.
             * @param topic const std::string& The name of the topic to advertise.
             * @param pub image_transport::Publisher& The image publisher to connect.
             * @param node_handle image_transport::ImageTransport& The image transport node to use.
             * @param queue_size int The queue size to use. Defaults to 20.
             */
            template <class MsgType>
            void create(const Topic& topic, image_transport::Publisher& pub, image_transport::ImageTransport& node_handle, int queue_size = 20);

        private:

            void connect_callback(const ros::SingleSubscriberPublisher& pub);
            void disconnect_callback(const ros::SingleSubscriberPublisher& pub);

            void connect_callback_image(const image_transport::SingleSubscriberPublisher& pub);
            void disconnect_callback_image(const image_transport::SingleSubscriberPublisher& pub);

            RosCallbackQueuePtr callback_queue_ptr;
    };



    template <class MsgType>
    void RosAsyncManager::create(const Topic& topic, ros::Publisher& pub, ros::NodeHandle& node_handle, int queue_size) {

        ros::SubscriberStatusCallback connect_cb =
                boost::bind(&RosAsyncManager::connect_callback, this, _1);

        ros::SubscriberStatusCallback disconnect_cb =
                boost::bind(&RosAsyncManager::disconnect_callback, this, _1);

        ros::AdvertiseOptions publisher_options =
                ros::AdvertiseOptions::create<MsgType>(topic, queue_size,
                connect_cb,
                disconnect_cb,
                ros::VoidPtr(),
                callback_queue_ptr.get());

        pub = node_handle.advertise(publisher_options);

    }

    template <class MsgType>
    void RosAsyncManager::create(const Topic& topic, image_transport::Publisher& pub, 
        image_transport::ImageTransport& node_handle, int queue_size) {

        image_transport::SubscriberStatusCallback connect_cb =
                boost::bind(&RosAsyncManager::connect_callback_image, this, _1);

        image_transport::SubscriberStatusCallback disconnect_cb =
                boost::bind(&RosAsyncManager::disconnect_callback_image, this, _1);


        pub = node_handle.advertise(topic, queue_size,
                connect_cb,
                disconnect_cb,
                ros::VoidPtr());

    }

	typedef std::shared_ptr<RosAsyncManager> RosAsyncPublisherPtr;
	typedef std::unique_ptr<RosAsyncManager> RosAsyncPublisherUniquePtr;

}


#endif
