#ifndef VDO_SLAM_SERALIZATION_H
#define VDO_SLAM_SERALIZATION_H

#include <yaml-cpp/yaml.h>
#include "vdo_slam/Params.h"
#include "vdo_slam/Scene.h"
#include "vdo_slam/utils/VdoUtils.h"

namespace YAML {
    template<>
    struct convert<VDO_SLAM::VdoParams> {
        static Node encode(const VDO_SLAM::VdoParams& rhs) {
            Node node;

            node["camera"]["fx"] = rhs.fx;
            node["camera"]["fy"] = rhs.fy;
            node["camera"]["cx"] = rhs.cx;
            node["camera"]["cy"] = rhs.cy;

            node["camera"]["k1"] = rhs.k1;
            node["camera"]["k2"] = rhs.k2;
            node["camera"]["p1"] = rhs.p1;
            node["camera"]["p2"] = rhs.p2;

            node["camera"]["fps"] = rhs.fps;
            node["camera"]["bf"] = rhs.bf;
            node["camera"]["depth_map_factor"] = rhs.depth_map_factor;

            node["image"]["width"] = rhs.width;
            node["image"]["height"] = rhs.height;
            node["image"]["RGB"] = rhs.RGB;


            node["system"]["data_code"] = rhs.data_code;
            node["system"]["sensor_type"] = VDO_SLAM::utils::sensor_to_string(rhs.sensor_type);
            node["system"]["thdepth_bg"] = rhs.thdepth_bg;
            node["system"]["thdepth_obj"] = rhs.thdepth_obj;
            node["system"]["max_track_points_bg"] = rhs.max_track_points_bg;
            node["system"]["max_track_points_obj"] = rhs.max_track_points_obj;
            node["system"]["sf_mg_thresh"] = rhs.sf_mg_thresh;
            node["system"]["sf_ds_thresh"] = rhs.sf_ds_thresh;

            node["system"]["window_size"] = rhs.window_size;
            node["system"]["overlap_size"] = rhs.overlap_size;

            node["system"]["use_sample_feature"] = rhs.use_sample_feature;
            node["system"]["n_features"] = rhs.n_features;
            node["system"]["scale_factor"] = rhs.scale_factor;
            node["system"]["n_levels"] = rhs.n_levels;         

            node["system"]["ini_th_fast"] = rhs.ini_th_fast;         
            node["system"]["min_th_fast"] = rhs.min_th_fast;            

            return node;
        }

    };

    template<>
    struct convert<VDO_SLAM::Time> {
        static Node encode(const VDO_SLAM::Time& rhs) {
            Node node;
            node["sec"] = rhs.sec;
            node["nsec"] = rhs.nsec;
            return node;
        }

    };


    template<>
    struct convert<VDO_SLAM::BoundingBox> {
        static Node encode(const VDO_SLAM::BoundingBox& rhs) {
            Node node;
            node["x"] = rhs.x;
            node["y"] = rhs.y;
            node["width"] = rhs.width;
            node["height"] = rhs.height;
            return node;
        }

    };

    template<>
    struct convert<VDO_SLAM::Odometry> {
        static Node encode(const VDO_SLAM::Odometry& rhs) {
            Node node;
            node["pose"]["translation"]["x"] = rhs.pose.translation()[0];
            node["pose"]["translation"]["y"] = rhs.pose.translation()[1];
            node["pose"]["translation"]["z"] = rhs.pose.translation()[2];

            node["pose"]["rotation"]["x"] = rhs.pose.rotation().x();
            node["pose"]["rotation"]["y"] = rhs.pose.rotation().y();
            node["pose"]["rotation"]["z"] = rhs.pose.rotation().z();
            node["pose"]["rotation"]["w"] = rhs.pose.rotation().w();

            node["twist"]["linear"]["x"] = rhs.twist.translation()[0];
            node["twist"]["linear"]["y"] = rhs.twist.translation()[1];
            node["twist"]["linear"]["z"] = rhs.twist.translation()[2];

            //for now this is just the way the odom messges are written becuase
            //we convert between ros and here so much
            node["twist"]["angular"]["x"] = rhs.twist.rotation().x();
            node["twist"]["angular"]["y"] = rhs.twist.rotation().y();
            node["twist"]["angular"]["z"] = rhs.twist.rotation().z();
            node["time"] = rhs.time;
            return node;
        }

    };


    template<>
    struct convert<VDO_SLAM::SceneObject> {
        static Node encode(const VDO_SLAM::SceneObject& rhs) {
            Node node;

            //this should be uint16_t but atm code in Tracking sets to floats and
            //currently dont want to change in case it introduces bugs

            //currently we do not encodce the image center becuase this only gets called in the viauzlizer
            //this is after the emssage gets converted to ROS (where we create the bounding box) and loose the 
            //center image data - but this information is encoded in the bounding box
            // node["image_center"]["u"] = rhs.center_image.at<float>(0, 0);
            // node["image_center"]["v"] = rhs.center_image.at<float>(1, 0);
            node["semantic_instance_index"] = rhs.semantic_instance_index;
            node["label"] = rhs.label;
            node["tracking_id"] = rhs.tracking_id;
            node["frame_id"] = rhs.frame_id;
            node["unique_id"] = rhs.unique_id;
            node["diff_time"] = rhs.timestamp;
            node["bounding_box"] = rhs.bounding_box;
            node["scene_time"] = rhs.scene_time;

            node["pose"]["translation"]["x"] = rhs.pose->translation()[0];
            node["pose"]["translation"]["y"] = rhs.pose->translation()[1];
            node["pose"]["translation"]["z"] = rhs.pose->translation()[2];

            node["pose"]["rotation"]["x"] = rhs.pose->rotation().x();
            node["pose"]["rotation"]["y"] = rhs.pose->rotation().y();
            node["pose"]["rotation"]["z"] = rhs.pose->rotation().z();
            node["pose"]["rotation"]["w"] = rhs.pose->rotation().w();

            node["twist"]["linear"]["x"] = rhs.twist->translation()[0];
            node["twist"]["linear"]["y"] = rhs.twist->translation()[1];
            node["twist"]["linear"]["z"] = rhs.twist->translation()[2];

            //for now this is just the way the odom messges are written becuase
            //we convert between ros and here so much
            node["twist"]["angular"]["x"] = rhs.twist->rotation().x();
            node["twist"]["angular"]["y"] = rhs.twist->rotation().y();
            node["twist"]["angular"]["z"] = rhs.twist->rotation().z();


            
            
            
            return node;
        }

    };

    template<>
    struct convert<VDO_SLAM::Scene> {
        static Node encode(const VDO_SLAM::Scene& rhs) {
            Node node;

            VDO_SLAM::Scene& scene_ref = const_cast<VDO_SLAM::Scene&>(rhs);
        
            for(VDO_SLAM::SceneObjectPtr& scene_objectPtr : scene_ref.get_scene_objects()) {
                node["objects"].push_back(*scene_objectPtr);
            }

            node["scene_time"] = rhs.scene_time;
            node["camera_pose"]["translation"]["x"] = rhs.pose->translation()[0];
            node["camera_pose"]["translation"]["y"] = rhs.pose->translation()[1];
            node["camera_pose"]["translation"]["z"] = rhs.pose->translation()[2];

            node["camera_pose"]["rotation"]["x"] = rhs.pose->rotation().x();
            node["camera_pose"]["rotation"]["y"] = rhs.pose->rotation().y();
            node["camera_pose"]["rotation"]["z"] = rhs.pose->rotation().z();
            node["camera_pose"]["rotation"]["w"] = rhs.pose->rotation().w();

            node["camera_twist"]["linear"]["x"] = rhs.twist->translation()[0];
            node["camera_twist"]["linear"]["y"] = rhs.twist->translation()[1];
            node["camera_twist"]["linear"]["z"] = rhs.twist->translation()[2];

            //for now this is just the way the odom messges are written becuase
            //we convert between ros and here so much
            node["camera_twist"]["angular"]["x"] = rhs.twist->rotation().x();
            node["camera_twist"]["angular"]["y"] = rhs.twist->rotation().y();
            node["camera_twist"]["angular"]["z"] = rhs.twist->rotation().z();

            //note we do not encode the rgb_frame (cv::Mat type). We can change this later
            
            return node;
        }

    };



} //namespace YAML




#endif