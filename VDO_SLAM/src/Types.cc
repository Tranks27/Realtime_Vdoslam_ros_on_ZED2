#include "vdo_slam/Types.h"
#include "vdo_slam/Macros.h"


VDO_SLAM::eSensor VDO_SLAM::param_to_sensor(const int sensor) {
    if (sensor == 0) {
        VDO_INFO_MSG("Using MONOCULAR sensor.");
        return VDO_SLAM::eSensor::MONOCULAR;
    }
    else if (sensor == 1) {
        VDO_INFO_MSG("Using STEREO sensor.");
        return VDO_SLAM::eSensor::STEREO;
    }
    else if (sensor == 2) {
        VDO_INFO_MSG("Using RGBD sensor.");
        return VDO_SLAM::eSensor::RGBD;
    }
    else {
        VDO_ERROR_MSG("Sensor type " << sensor << " is invalid.");
        return VDO_SLAM::eSensor::INVALID;
    }

}
