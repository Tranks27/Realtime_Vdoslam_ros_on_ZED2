#include "vdo_slam/Scene.h"
#include "vdo_slam/Converter.h"
#include "vdo_slam/Macros.h"
#include "vdo_slam/utils/VdoUtils.h"
#include "vdo_slam/utils/Types.h"
#include "vdo_slam/map/Map.h"
#include "vdo_slam/map/MapObject.h"





namespace VDO_SLAM {
    std::ostream& operator << (std::ostream& output, const VDO_SLAM::SceneObject& object) {
        output << "SceneObject [pose:\n: " << *object.pose;
        output << "\n Velocity:\n: " << *object.twist;
        output << "\nLabel: " << object.label<< " Semantic Instance: " << object.semantic_instance_index << " tracking ID " << object.tracking_id << " ]";

        return output;
    }


    bool VDO_SLAM::SceneObject::update_from_map(const Map* map) {

        
    }



    Scene::Scene() {

        cv::Mat identity = utils::homogenous_identity();
        pose_from_homogenous_mat(identity);
        twist_from_homogenous_mat(identity);

    }

    Scene::Scene(int frame_id_, const Time& time_)
        :   frame_id(frame_id_),
            scene_time(time_) {
        
            cv::Mat identity = utils::homogenous_identity();
            pose_from_homogenous_mat(identity);
            twist_from_homogenous_mat(identity);
        }


    void Scene::add_scene_object(SceneObjectPtr& _object) {
        _object->scene_time = scene_time;
        _object->timestamp = timestamp;
        _object->frame_id = frame_id;
        scene_objects.push_back(_object);
    }

    bool Scene::update_from_map(const Map* map) {
        cv::Mat rf_camera_pose = map->vmCameraPose_RF[frame_id-1];
        // utils::image_to_global_coordinates(rf_camera_pose, rf_camera_pose);
        pose_from_homogenous_mat(rf_camera_pose);
        return true;
    }


    std::vector<std::shared_ptr<SceneObject>>& Scene::get_scene_objects() {
        return scene_objects;
    }

    int Scene::scene_objects_size() {
        return scene_objects.size();
    }

    
    int Scene::get_id() {
        return frame_id;
    }
    
    
    double Scene::get_timestamp() {
        return timestamp;
    }


}

