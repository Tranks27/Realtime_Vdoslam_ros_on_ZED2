#ifndef VDO_SLAM_VISUALIZER_OUTPUT_BASE_H
#define VDO_SLAM_VISUALIZER_OUTPUT_BASE_H

#include <memory>

namespace VDO_SLAM {

    struct VisualizerOutput {

        virtual ~VisualizerOutput() = default;        


    };

    typedef std::shared_ptr<VisualizerOutput> VisualizerOutputPtr;
    typedef std::unique_ptr<VisualizerOutput> VisualizerOutputUniquePtr;

}

#endif

