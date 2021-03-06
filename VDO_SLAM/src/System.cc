/**
* This file is part of VDO-SLAM.
*
* Copyright (C) 2019-2020 Jun Zhang <jun doc zhang2 at anu dot edu doc au> (The Australian National University)
* For more information see <https://github.com/halajun/VDO_SLAM>
*
**/

// #include "System.h"
// #include "Converter.h"
#include "vdo_slam/Scene.h"
#include "vdo_slam/System.h"
#include "vdo_slam/Tracking.h"
#include "vdo_slam/Macros.h"
#include "vdo_slam/utils/Types.h"
#include "vdo_slam/utils/VdoUtils.h"

#include <thread>
#include <iomanip>


#include <unistd.h>

namespace VDO_SLAM {


    System::System(const VdoParams& params) {
        //Create the Map
        mpMap = new Map();


        //Initialize the Tracking thread
        //(it will live in the main thread of execution, the one that called this constructor)
        mpTracker = new Tracking(mpMap, params);
    }


    System::System(const string &strSettingsFile, const eSensor sensor):mSensor(sensor)
    {
        /**********not using this constructor***********
        
        // ===== output welcome message ======
        cout << endl << 
        " ----------------------------------------------------------------------------" << endl <<
        "| VDO-SLAM Copyright (C) 2019-2020 Jun Zhang, Australian National University.|" << endl <<
        "| This program comes with ABSOLUTELY NO WARRANTY;                            |" << endl  <<
        "| This is free software, and you are welcome to redistribute it              |" << endl <<
        "| under certain conditions; See LICENSE.txt.                                 |" << endl << 
        " ----------------------------------------------------------------------------" << endl;

        // Check settings file
        cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
        cerr << "Failed to open settings file at: " << strSettingsFile << endl;
        exit(-1);
        }

        //Create the Map
        mpMap = new Map();

        //Initialize the Tracking thread
        //(it will live in the main thread of execution, the one that called this constructor)
        mpTracker = new Tracking(mpMap, strSettingsFile, mSensor);

        */
    }




    std::pair<SceneType, std::shared_ptr<Scene>> System::TrackRGBD(const cv::Mat &imRGB, cv::Mat &imD, const cv::Mat &imFlow, const cv::Mat &maskSEM,
                            const Time& time_, const double &timestamp, cv::Mat &imTraj, const int &nImage) {
        // if(mSensor!=RGBD)
        // {
        //     cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
        //     exit(-1);
        // }

        std::pair<SceneType, std::shared_ptr<Scene>> result_pair = mpTracker->GrabImageRGBD(imRGB,imD,imFlow,maskSEM, time_,timestamp,imTraj,nImage);
        slam_scene_queue_.push(result_pair.second);
        return result_pair;

    }

    void System::shutdown() {

        // if(satistics_) {
        //     satistics_->writeStatistics();
        // }
        VDO_INFO_MSG("Shutting down SLAM system");

    }


    bool System::construct_scenes(std::vector<SlamScenePtr>& scenes, int back_frame_id) {
        // const int N = mpMap->vpFeatSta.size();
        //need to check scenes.size() == vmCameraPose_RF.size()
        assert(scenes.size() == mpMap->vmCameraPose_RF.size());
        const int N = scenes.size();

        if (N < 2) {
            return false;

        }
        int n_frames = N;
        if(back_frame_id == -1) {
            n_frames = N;
            VDO_INFO_MSG(n_frames);
        }
        else if(back_frame_id > N) {
            VDO_WARN_MSG("Back frame ID " << back_frame_id << " greater than size of map " << N);
            n_frames = N;
        }
        else {
            n_frames = N - back_frame_id;
        }

        VDO_INFO_MSG("Constructing scenes using last " << n_frames << "/" << N);
        // for(int i = 0; i < N; i++) {
        //     mpMap->update_from_map()
        // }
        for(SlamScenePtr& scene : scenes) {
            mpMap->update_from_map(scene.get());
        }
    }

    void System::SaveResultsIJRR2020(const string &filename)
    {
        cout << endl << "Saving Results into TXT File..." << endl;

        // *******************************************************************************************************
        // ***************************************** SAVE OBJ SPEED **********************************************
        // *******************************************************************************************************

        ofstream save_objmot, save_objmot_gt;
        string path_objmot = filename + "obj_mot_rgbd_new.txt";
        string path_objmot_gt = filename + "obj_mot_gt.txt";
        save_objmot.open(path_objmot.c_str(),ios::trunc);
        save_objmot_gt.open(path_objmot_gt.c_str(),ios::trunc);

        int start_frame = 0;
        // main loop
        for (int i = 0; i < mpMap->vmRigidMotion.size(); ++i)
        {
            if (mpMap->vmRigidMotion[i].size()>1)
            {
                for (int j = 1; j < mpMap->vmRigidMotion[i].size(); ++j)
                {
                    save_objmot << start_frame+i+1 << " " <<  mpMap->vnRMLabel[i][j] << " " << fixed << setprecision(9) <<
                                    mpMap->vmRigidMotion[i][j].at<float>(0,0) << " " << mpMap->vmRigidMotion[i][j].at<float>(0,1)  << " " << mpMap->vmRigidMotion[i][j].at<float>(0,2) << " "  << mpMap->vmRigidMotion[i][j].at<float>(0,3) << " " <<
                                    mpMap->vmRigidMotion[i][j].at<float>(1,0) << " " << mpMap->vmRigidMotion[i][j].at<float>(1,1)  << " " << mpMap->vmRigidMotion[i][j].at<float>(1,2) << " "  << mpMap->vmRigidMotion[i][j].at<float>(1,3) << " " <<
                                    mpMap->vmRigidMotion[i][j].at<float>(2,0) << " " << mpMap->vmRigidMotion[i][j].at<float>(2,1)  << " " << mpMap->vmRigidMotion[i][j].at<float>(2,2) << " "  << mpMap->vmRigidMotion[i][j].at<float>(2,3) << " " <<
                                    0.0 << " " << 0.0 << " " << 0.0 << " " << 1.0 << endl;

                    save_objmot_gt << start_frame+i+1 << " " <<  mpMap->vnRMLabel[i][j] << " " << fixed << setprecision(9) <<
                                    mpMap->vmRigidMotion_GT[i][j].at<float>(0,0) << " " << mpMap->vmRigidMotion_GT[i][j].at<float>(0,1)  << " " << mpMap->vmRigidMotion_GT[i][j].at<float>(0,2) << " "  << mpMap->vmRigidMotion_GT[i][j].at<float>(0,3) << " " <<
                                    mpMap->vmRigidMotion_GT[i][j].at<float>(1,0) << " " << mpMap->vmRigidMotion_GT[i][j].at<float>(1,1)  << " " << mpMap->vmRigidMotion_GT[i][j].at<float>(1,2) << " "  << mpMap->vmRigidMotion_GT[i][j].at<float>(1,3) << " " <<
                                    mpMap->vmRigidMotion_GT[i][j].at<float>(2,0) << " " << mpMap->vmRigidMotion_GT[i][j].at<float>(2,1)  << " " << mpMap->vmRigidMotion_GT[i][j].at<float>(2,2) << " "  << mpMap->vmRigidMotion_GT[i][j].at<float>(2,3) << " " <<
                                    0.0 << " " << 0.0 << " " << 0.0 << " " << 1.0 << endl;
                }
            }
        }

        save_objmot.close();
        save_objmot_gt.close();

        // *******************************************************************************************************
        // ***************************************** SAVE CAMERA TRAJETORY ***************************************
        // *******************************************************************************************************

        std::vector<cv::Mat> CamPose_ini = mpMap->vmCameraPose;

        ofstream save_traj_ini;
        string path_ini = filename + "initial_rgbd_new.txt";
        // cout << path_ini << endl;
        save_traj_ini.open(path_ini.c_str(),ios::trunc);

        for (int i = 0; i < CamPose_ini.size(); ++i)
        {
            save_traj_ini << start_frame+i << " " << fixed << setprecision(9) << CamPose_ini[i].at<float>(0,0) << " " << CamPose_ini[i].at<float>(0,1)  << " " << CamPose_ini[i].at<float>(0,2) << " "  << CamPose_ini[i].at<float>(0,3) << " " <<
                            CamPose_ini[i].at<float>(1,0) << " " << CamPose_ini[i].at<float>(1,1)  << " " << CamPose_ini[i].at<float>(1,2) << " "  << CamPose_ini[i].at<float>(1,3) << " " <<
                            CamPose_ini[i].at<float>(2,0) << " " << CamPose_ini[i].at<float>(2,1)  << " " << CamPose_ini[i].at<float>(2,2) << " "  << CamPose_ini[i].at<float>(2,3) << " " <<
                            0.0 << " " << 0.0 << " " << 0.0 << " " << 1.0 << endl;
        }

        save_traj_ini.close();

        // ------------------------------------------------------------------------------------------------------------

        std::vector<cv::Mat> CamPose_ref = mpMap->vmCameraPose_RF;

        ofstream save_traj_ref;
        string path_ref = filename + "refined_rgbd_new.txt";
        save_traj_ref.open(path_ref.c_str(),ios::trunc);

        for (int i = 0; i < CamPose_ref.size(); ++i)
        {
            save_traj_ref << start_frame+i << " " << fixed << setprecision(9) << CamPose_ref[i].at<float>(0,0) << " " << CamPose_ref[i].at<float>(0,1)  << " " << CamPose_ref[i].at<float>(0,2) << " "  << CamPose_ref[i].at<float>(0,3) << " " <<
                            CamPose_ref[i].at<float>(1,0) << " " << CamPose_ref[i].at<float>(1,1)  << " " << CamPose_ref[i].at<float>(1,2) << " "  << CamPose_ref[i].at<float>(1,3) << " " <<
                            CamPose_ref[i].at<float>(2,0) << " " << CamPose_ref[i].at<float>(2,1)  << " " << CamPose_ref[i].at<float>(2,2) << " "  << CamPose_ref[i].at<float>(2,3) << " " <<
                            0.0 << " " << 0.0 << " " << 0.0 << " " << 1.0 << endl;
        }

        save_traj_ref.close();

        // ------------------------------------------------------------------------------------------------------------

        std::vector<cv::Mat> CamPose_gt = mpMap->vmCameraPose_GT;

        ofstream save_traj_gt;
        string path_gt = filename + "cam_pose_gt.txt";
        save_traj_gt.open(path_gt.c_str(),ios::trunc);

        for (int i = 0; i < CamPose_gt.size(); ++i)
        {
            save_traj_gt << start_frame+i << " " << fixed << setprecision(9) << CamPose_gt[i].at<float>(0,0) << " " << CamPose_gt[i].at<float>(0,1)  << " " << CamPose_gt[i].at<float>(0,2) << " "  << CamPose_gt[i].at<float>(0,3) << " " <<
                            CamPose_gt[i].at<float>(1,0) << " " << CamPose_gt[i].at<float>(1,1)  << " " << CamPose_gt[i].at<float>(1,2) << " "  << CamPose_gt[i].at<float>(1,3) << " " <<
                            CamPose_gt[i].at<float>(2,0) << " " << CamPose_gt[i].at<float>(2,1)  << " " << CamPose_gt[i].at<float>(2,2) << " "  << CamPose_gt[i].at<float>(2,3) << " " <<
                            0.0 << " " << 0.0 << " " << 0.0 << " " << 1.0 << endl;
        }

        save_traj_gt.close();

        // ------------------------------------------------------------------------------------------------------------

        // std::vector<cv::Mat> CamPose_orb = mpMap->vmCameraPose_orb;

        // ofstream save_traj_orb;
        // string path_orb = filename + "orb.txt";
        // save_traj_orb.open(path_orb.c_str(),ios::trunc);


        // for (int i = 0; i < CamPose_orb.size(); ++i)
        // {
        //     save_traj_orb  << fixed << setprecision(9) << CamPose_orb[i].at<float>(0,0) << " " << CamPose_orb[i].at<float>(0,1)  << " " << CamPose_orb[i].at<float>(0,2) << " "  << CamPose_orb[i].at<float>(0,3) << " " <<
        //                       CamPose_orb[i].at<float>(1,0) << " " << CamPose_orb[i].at<float>(1,1)  << " " << CamPose_orb[i].at<float>(1,2) << " "  << CamPose_orb[i].at<float>(1,3) << " " <<
        //                       CamPose_orb[i].at<float>(2,0) << " " << CamPose_orb[i].at<float>(2,1)  << " " << CamPose_orb[i].at<float>(2,2) << " "  << CamPose_orb[i].at<float>(2,3) << " " <<
        //                       0.0 << " " << 0.0 << " " << 0.0 << " " << 1.0 << endl;
        // }

        // save_traj_orb.close();

        // *******************************************************************************************************
        // ***************************************** SAVE TIME ANALYSIS ******************************************
        // *******************************************************************************************************

        std::vector<std::vector<float> > All_timing = mpMap->vfAll_time;
        std::vector<float> LBA_timing = mpMap->fLBA_time;

        int obj_time_count=0;

        // all tracking components
        std::vector<float> avg_timing(All_timing[0].size(),0);
        for (int i = 0; i < All_timing.size(); ++i)
            for (int j = 0; j < All_timing[i].size(); ++j)
            {
                if (j==3 && All_timing[i][j]!=0)
                {
                    avg_timing[j] = avg_timing[j] + All_timing[i][j];
                    obj_time_count = obj_time_count + 1;
                }
                else
                    avg_timing[j] = avg_timing[j] + All_timing[i][j];
            }

        cout << "Time of all components: " << endl;
        for (int j = 0; j < avg_timing.size(); ++j)
        {
            if (j==3)
                cout << "(" << j << "): " <<  avg_timing[j]/obj_time_count << " ";
            else
                cout << "(" << j << "): " <<  avg_timing[j]/All_timing.size() << " ";
        }
        cout << endl;

        // local bundle adjustment
        float avg_lba_timing = 0;
        for (int i = 0; i < LBA_timing.size(); ++i)
            avg_lba_timing = avg_lba_timing + LBA_timing[i];
        cout << "Time of local bundle adjustment: " << avg_lba_timing/LBA_timing.size() << endl;



        // ************************************************************************************************************
        // ************************************************************************************************************

    }



} //namespace VDO_SLAM
