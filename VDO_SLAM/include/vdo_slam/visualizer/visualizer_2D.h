#ifndef VDO_SLAM_VISUALIZER_2D_H
#define VDO_SLAM_VISUALIZER_2D_H

#include "vdo_slam/visualizer/visualizer_base.h"
#include "vdo_slam/visualizer/visualizer_params.h"
#include "vdo_slam/visualizer/visualizer_output_base.h"
#include "vdo_slam/visualizer/colour.h"
#include "vdo_slam/Scene.h"
#include "vdo_slam/utils/Types.h"

#include <vdo_slam_g2o/types/se3quat.h>
#include <opencv2/opencv.hpp>
#include <mutex>


namespace VDO_SLAM {


    struct VisualizerOutput2D : public VisualizerOutput {
        cv::Mat bounding_box_display_;
        cv::Mat object_point_display_;

    };


    typedef std::shared_ptr<VisualizerOutput2D> VisualizerOutput2DPtr;
    typedef std::unique_ptr<VisualizerOutput2D> VisualizerOutput2DUniquePtr;


    class Visualizer2D : public VisualizerBase {

        public:
            Visualizer2D(VisualizerParamsPtr& params_);

            VisualizerOutputPtr spinOnce(SlamScenePtr& slam_scene_) override;
            void shutdown() override;

            void update_gt_odom(Odometry& odom);

        protected:
            void update_object_points_display(SlamScenePtr& slam_scene_);
            void update_projected_box_display(SlamScenePtr& slam_scene_);

        private:
            cv::Mat overlay_scene_image(const SlamScenePtr& slam_scene_);



            VisualizerOutput2DUniquePtr output_viz_;
            std::mutex display_lock;

            SensorDataWrapper<Odometry> odom_gt;

    };
}

#endif
