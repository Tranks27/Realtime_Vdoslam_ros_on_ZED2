/**
* This file is part of VDO-SLAM.
*
* Copyright (C) 2019-2020 Jun Zhang <jun doc zhang2 at anu dot edu doc au> (The Australian National University)
* For more information see <https://github.com/halajun/VDO_SLAM>
*
**/

#ifndef VDO_SLAM_OPTIMIZER_H
#define VDO_SLAM_OPTIMIZER_H

#include "vdo_slam/map/Map.h"
#include "vdo_slam/Frame.h"
#include <vdo_slam_g2o/types/types_six_dof_expmap.h>


namespace VDO_SLAM
{

using namespace std;

class Optimizer
{
public:

    int static PoseOptimizationNew(Frame *pCurFrame, Frame *pLastFrame, vector<int> &TemperalMatch);
    int static PoseOptimizationFlow2Cam(Frame *pCurFrame, Frame *pLastFrame, vector<int> &TemperalMatch);
    cv::Mat static PoseOptimizationObjMot(Frame *pCurFrame, Frame *pLastFrame, const vector<int> &ObjId, std::vector<int> &InlierID);
    cv::Mat static PoseOptimizationFlow2(Frame *pCurFrame, Frame *pLastFrame, const vector<int> &ObjId, std::vector<int> &InlierID);
    void static FullBatchOptimization(Map* pMap, const cv::Mat Calib_K);
    void static PartialBatchOptimization(Map* pMap, const cv::Mat Calib_K, const int WINDOW_SIZE);
    
    // convert pixel coords to World coords
    cv::Mat static Get3DinWorld(const cv::KeyPoint &Feats2d, const float &Dpts, const cv::Mat &Calib_K, const cv::Mat &CameraPose);
    
    // convert pixel coords to camera coords
    cv::Mat static Get3DinCamera(const cv::KeyPoint &Feats2d, const float &Dpts, const cv::Mat &Calib_K);

};

} //namespace VDO_SLAM

#endif // OPTIMIZER_H
