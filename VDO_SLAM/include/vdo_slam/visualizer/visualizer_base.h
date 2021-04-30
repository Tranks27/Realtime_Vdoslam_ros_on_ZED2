#ifndef VDO_SLAM_VISUALIZER_BASE_H
#define VDO_SLAM_VISUALIZER_BASE_H

#include "vdo_slam/visualizer/visualizer_params.h"
#include "vdo_slam/visualizer/visualizer_output_base.h"
#include "vdo_slam/visualizer/colour.h"
#include "vdo_slam/Scene.h"

#include <functional>

namespace VDO_SLAM {

    class VisualizerBase {

        typedef std::function<void()> RenderFunc;

        public:
            VisualizerBase(VisualizerParamsPtr& params_);
            virtual ~VisualizerBase() = default;

            /**
             * @brief Updates any visualization with the new slam scene
             * 
             * @param slam_scene_ 
             */
            virtual VisualizerOutputPtr spinOnce(SlamScenePtr& slam_scene_) = 0;

            void connect_render_func(RenderFunc&& func_);

        protected:

            VisualizerParamsPtr params;
            RenderFunc render_func;

            ColourManager color_manager_;

        

    };
}


#endif