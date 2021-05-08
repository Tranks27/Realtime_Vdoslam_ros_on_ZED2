#ifndef VDO_SLAM_PARAMS_H
#define VDO_SLAM_PARAMS_H

#include "vdo_slam/utils/Types.h"
#include <memory>

namespace VDO_SLAM {

    /**
     * @brief Define a struct that contains all the params needed to initalise the System. Can be used
     * as a replacement for the yaml settings file. 
     * 
     */
    struct VdoParams {

        //Camera intrinsics
        float fx;
        float fy;
        float cx;
        float cy;

        //distortion params
        float k1;
        float k2;
        float p1;
        float p2;
        float p3 = 0;

        //image size
        int width;
        int height;

        int fps;

        // stero baseline times fx
        float bf;

        // Color order of the images (0: BGR, 1: RGB. It is ignored if images are grayscale)
        int RGB;

        //System Params


        int data_code;

        //VDO_SLAM::eSensor
        eSensor sensor_type;

        float depth_map_factor;

        // Close/Far Depth threshold
        float thdepth_bg;
        float thdepth_obj;

        // Max Tracking Points on Background and Object in each frame
        float max_track_points_bg;
        float max_track_points_obj;

        // Scene Flow Magnitude and Distribution Threshold
        float sf_mg_thresh;
        float sf_ds_thresh;

        // Window Size and Overlapping Size for Local Batch Optimization
        int window_size;
        int overlap_size;

        //  Use sampled feature or detected feature for background (1: sampled, 0: detected)
        int use_sample_feature;

        // Orb Params

        // Number of features per image
        int n_features;

        // Scale factor between levels in the scale pyramid
        float scale_factor;

        // Number of levels in the scale pyramid
        int n_levels;

        // Fast threshold
        int ini_th_fast;
        int min_th_fast;


        //statistic params
        bool shoud_write; //if true will write to file in output_results/

    };

    typedef const std::shared_ptr<VdoParams> VdoParamsConstPtr;
    typedef std::shared_ptr<VdoParams> VdoParamsPtr;

}

#endif