#include "vdo_slam/visualizer/visualizer_base.h"

namespace VDO_SLAM {

    VisualizerBase::VisualizerBase(VisualizerParamsPtr& params_)
        :   params(params_),
            color_manager_(params->classes_filepath) {}

    void VisualizerBase::connect_render_func(RenderFunc&& func_) {
        render_func = func_;
    }
}