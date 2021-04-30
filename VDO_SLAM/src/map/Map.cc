/**
* This file is part of VDO-SLAM.
*
* Copyright (C) 2019-2020 Jun Zhang <jun doc zhang2 at anu dot edu doc au> (The Australian National University)
* For more information see <https://github.com/halajun/VDO_SLAM>
*
**/

#include <iostream>

#include "vdo_slam/map/Map.h"
#include "vdo_slam/map/MapObject.h"

namespace VDO_SLAM
{

    Map::Map():mnMaxKFid(0),mnBigChangeIdx(0)
    {
    }

    bool Map::update_from_map(MapObject* map_object) {
        return map_object->update_from_map(this);
    }


    void Map::reset() {
        std::cout << "Resetting map" << std::endl;
        vpFeatSta.clear();
        vfDepSta.clear();
        vp3DPointSta.clear();
        vnAssoSta.clear();
        TrackletSta.clear();
        vpFeatDyn.clear();
        vfDepDyn.clear();
        vp3DPointDyn.clear();
        vnAssoDyn.clear();
        vnFeatLabel.clear();
        TrackletDyn.clear();
        nObjID.clear();
        vmCameraPose.clear();
        vmCameraPose_RF.clear();
        vmCameraPose_GT.clear();
        vmRigidCentre.clear();
        vmRigidMotion.clear();
        vmObjPosePre.clear();
        vmRigidMotion_RF.clear();
        vmRigidMotion_GT.clear();
        vfAllSpeed_GT.clear();
        vnRMLabel.clear();
        vnSMLabel.clear();
        vnSMLabelGT.clear();
        vbObjStat.clear();
        vnObjTraTime.clear();
        nObjTraCount.clear();
        nObjTraCountGT.clear();
        nObjTraSemLab.clear();
        fLBA_time.clear();
        vfAll_time.clear();

        mnMaxKFid = 0;
        mnBigChangeIdx = 0;
    }

} //namespace VDO_SLAM
