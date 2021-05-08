#ifndef VDO_SLAM_SCENE_H
#define VDO_SLAM_SCENE_H

#include <opencv2/core/types.hpp>
#include <opencv2/core/core.hpp>
#include <string>
#include <vector>
#include <iostream>
#include <memory>
#include <opencv2/core/core.hpp>

#include <eigen3/Eigen/Dense>
#include  <vdo_slam_g2o/types/types_seven_dof_expmap.h>

#include "vdo_slam/utils/Types.h"
#include "vdo_slam/map/MapObject.h"

namespace VDO_SLAM
{

    struct SceneObject : public EuclideanObject, public MapObject {

        cv::Mat center_image; //center in the 2D image plane in the form (u, v)
        int semantic_instance_index; 
        std::string label;
        int tracking_id; 
        int frame_id;
        int unique_id;
        double timestamp = -1;
        BoundingBox bounding_box;
        Time scene_time;
        // std::vector<cv::KeyPoint> keypoints;

        virtual ~SceneObject() {}


        bool update_from_map(const Map* map) override;

        friend std::ostream &operator << (std::ostream& output, const SceneObject& object);


        template<class T>
        inline static std::shared_ptr<SceneObject> create(T& t);

        template<class T>
        inline T convert();
        
    };

    class Scene : public EuclideanObject, public MapObject {
        
        public:
            Scene();
            Scene(int frame_id_, const Time& time_);
            virtual ~Scene() {}

            void add_scene_object(std::shared_ptr<SceneObject>& _object);
            std::vector<std::shared_ptr<SceneObject>>& get_scene_objects();
            bool update_from_map(const Map* map) override;

            int scene_objects_size();
            int get_id();
            double get_timestamp();

            cv::Mat rgb_frame;
            Time scene_time;

            template<class T>
            inline static std::shared_ptr<Scene> create(T& t);

            template<class T>
            inline T convert();



        protected:
            std::vector<std::shared_ptr<SceneObject>> scene_objects;
            int frame_id;
            double timestamp;
    };

    typedef std::shared_ptr<SceneObject> SceneObjectPtr;
    typedef std::unique_ptr<SceneObject> SceneObjectUniquePtr;


    typedef std::shared_ptr<Scene> SlamScenePtr;
    typedef std::unique_ptr<Scene> SlamSceneUniquePtr;

    
} // namespace VDO_SLAM


#endif
