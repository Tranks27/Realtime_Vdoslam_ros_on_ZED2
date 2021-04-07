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

#include "Tracking.h"
#include "Map.h"
#include "Scene.h"
#include "Params.h"
#include "Types.h"

using namespace std;


namespace VDO_SLAM {



    class Map;
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
        std::unique_ptr<Scene> TrackRGBD(const cv::Mat &im, cv::Mat &depthmap, const cv::Mat &flowmap, const cv::Mat &masksem,
                        const cv::Mat &mTcw_gt, const vector<vector<float> > &vObjPose_gt, const double &timestamp,
                        cv::Mat &imTraj, const int &nImage);

        void SaveResultsIJRR2020(const string &filename);

    private:

        // Input sensor
        VDO_SLAM::eSensor mSensor;

        // Map structure.
        VDO_SLAM::Map* mpMap;

        // Tracker. It receives a frame and computes the associated camera pose.
        VDO_SLAM::Tracking* mpTracker;

    };

}// namespace VDO_SLAM

#endif // SYSTEM_H
