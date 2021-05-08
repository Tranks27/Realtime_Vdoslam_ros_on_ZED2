#ifndef VDO_SLAM_DATA_PROVIDER_INTERFACE
#define VDO_SLAM_DATA_PROVIDER_INTERFACE

#include "vdo_slam/Macros.h"
#include "vdo_slam/utils/Types.h"

#include <functional>

typedef std::function<void(const VDO_SLAM::FrameInput&)> FrameInputCallback;
typedef std::function<void(const VDO_SLAM::Odometry&)> OdometryCallback;

namespace VDO_SLAM {
    

    class DataProviderInterface {


        public:
            DataProviderInterface() = default;
            virtual ~DataProviderInterface();

            ~DataProviderInterface() {}
            virtual bool spin() = 0;
            virtual bool shutdown() = 0;



            inline void registerFrameCallback(const FrameInputCallback& callback) {
                frame_input_callback_ = callback;
            }

            inline void registerOdometryCallback(const OdometryCallback& callback) {
                odom_gt_callback_ = callback;
            }


        protected:
            FrameInputCallback frame_input_callback_;
            OdometryCallback odom_gt_callback_;

    };
}


#endif