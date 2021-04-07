#ifndef VDO_SLAM_TYPES_H
#define VDO_SLAM_TYPES_H


namespace VDO_SLAM {

     // Input sensor
    enum eSensor {
        MONOCULAR=0,
        STEREO=1, //confusing between stereo and RGBD, for now use RGBD?
        RGBD=2,
        INVALID=3
    };

    /**
     * @brief Converts an integer to its associated eSensor type
     * 
     * MONOCULAR = 0
     * STEREO = 1
     * RGBD = 2
     * INVALID = 3 (this will be returned if sensor is not 0, 1 or 2.)
     * 
     * @param sensor 
     * @return eSensor 
     */
    eSensor param_to_sensor(const int sensor);
};


#endif
