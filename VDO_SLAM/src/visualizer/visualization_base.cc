#include "vdo_slam/visualizer/visualizer_base.h"
#include "vdo_slam/utils/statistics.h"

namespace VDO_SLAM {

    VisualizerBase::VisualizerBase(VisualizerParamsPtr& params_)
        :   params(params_),
            color_manager_(params->classes_filepath) {

                statistics_manager_ = std::make_shared<StatisticsManager>(true);
                statistics_manager_->printStatistics();


            }

    void VisualizerBase::connect_render_func(RenderFunc&& func_) {
        render_func = func_;
    }
}