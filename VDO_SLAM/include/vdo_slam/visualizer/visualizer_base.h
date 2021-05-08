#ifndef VDO_SLAM_VISUALIZER_BASE_H
#define VDO_SLAM_VISUALIZER_BASE_H

#include "vdo_slam/visualizer/visualizer_params.h"
#include "vdo_slam/visualizer/visualizer_output_base.h"
#include "vdo_slam/visualizer/colour.h"
#include "vdo_slam/utils/VdoUtils.h"
#include "vdo_slam/utils/statistics.h"
#include "vdo_slam/Scene.h"

#include <functional>
#include <memory>

namespace VDO_SLAM {

    template<typename SensorData>
    class SensorDataWrapper {

        public:
            SensorDataWrapper(SensorData& data_) {
                data =  make_unique<SensorData>(data);
                first_data = make_unique<SensorData>(data);
                is_first_ = false;
            }

            SensorDataWrapper(std::unique_ptr<SensorData>& data_ptr_) {
                data = std::move(data_ptr_);
                first_data = std::move(data_ptr_);
                is_first_ = false;
            }

            SensorDataWrapper() {
                is_first_ = true;
            }

            inline bool is_first() {return is_first_; }

            std::unique_ptr<SensorData> data;
            std::unique_ptr<SensorData> first_data;
            bool is_first_;


    };

    class VisualizerBase {

        typedef std::function<void()> RenderFunc;

        public:
            VisualizerBase(VisualizerParamsPtr& params_);
            virtual ~VisualizerBase() = default;

            /**
             * @brief Closes and GUI functionality and writes loggers to file if necessary;
             * 
             */
            virtual void shutdown() = 0;

            /**
             * @brief Updates any visualization with the new slam scene
             * 
             * @param slam_scene_ 
             */
            virtual VisualizerOutputPtr spinOnce(SlamScenePtr& slam_scene_) = 0;

            //currently unimplemented
            void connect_render_func(RenderFunc&& func_);

        protected:

            VisualizerParamsPtr params;
            RenderFunc render_func;

            ColourManager color_manager_;

            //Is here for now just so we can get results
            StatisticsManagerPtr statistics_manager_;

        

    };
}


#endif