/**
* This file is part of VDO-SLAM.
*
* Copyright (C) 2019-2020 Jun Zhang <jun doc zhang2 at anu dot edu doc au> (The Australian National University)
* For more information see <https://github.com/halajun/VDO_SLAM>
*
**/


#ifndef VDO_SLAM_SYSTEM_H
#define VDO_SLAM_SYSTEM_H

#include <string>
#include <thread>
#include <memory>
#include <opencv2/core/core.hpp>

#include "vdo_slam/Scene.h"
#include "vdo_slam/Params.h"
#include "vdo_slam/map/Map.h"
#include "vdo_slam/Tracking.h"
#include "vdo_slam/utils/Types.h"




using namespace std;


namespace VDO_SLAM {


    class Tracking;


    class System
    {

    public:

        // Initialize the SLAM system.
        System(const string &strSettingsFile, const eSensor sensor);
        System(const VdoParams& params);


        // Process the given rgbd frame. Depthmap must be registered to the RGB frame.
        // Input image: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
        // Input depthmap: Float (CV_32F).
        // Returns the camera pose (empty if tracking fails).
        std::pair<SceneType, std::shared_ptr<Scene>> TrackRGBD(const cv::Mat &imRGB, cv::Mat &imD, const cv::Mat &imFlow, const cv::Mat &maskSEM,
                            const Time& time_, const double &timestamp, cv::Mat &imTraj, const int &nImage);


        /**
         * @brief Constructs a vector of Scenes directorly from the Map. Used when an optimization 
         * ocurrs and all the scenes back time t need to be updated.
         * 
         * @param back_frame_id How many scenes, starting from the most recent one should be reconstructed from the 
         * map. 0 should result in a vector of length 1 and resulting scene should be the same as the Scene returned
         * from TrackRGBD (value only, not ptr). -1 (Default) means all scenes in the map. If it is greater than the number of frames
         * saved it will default to all the scenes and a warning will be raised.
         * 
         * @return std::vector<std::shared_ptr<Scene>> 
         */
        bool construct_scenes(std::vector<SlamScenePtr>& scenes, int back_frame_id = -1);

        

        void SaveResultsIJRR2020(const string &filename);

    private:

        // Input sensor
        eSensor mSensor;

        // Map structure.
        Map* mpMap;

        // Tracker. It receives a frame and computes the associated camera pose.
        Tracking* mpTracker;

    };

}// namespace VDO_SLAM

#endif // SYSTEM_H
