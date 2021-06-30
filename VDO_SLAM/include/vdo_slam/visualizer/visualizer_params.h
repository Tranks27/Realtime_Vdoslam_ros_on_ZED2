#ifndef VDO_SLAM_VISUALIZER_PARAMS_H
#define VDO_SLAM_VISUALIZER_PARAMS_H

#include <memory>
#include <string>

namespace VDO_SLAM {

    struct VisualizerParams {

        //to actually show the window using opencv
        bool display_window;

        //full file path to the classes.csv file. Used for colour manager
        std::string classes_filepath;

        //used for display mat
        // int x_offset = 150;
        // int y_offset = 150;
        int x_offset = 300;
        int y_offset = 100;
        int scale = 6; //6

    };

    typedef std::shared_ptr<VisualizerParams> VisualizerParamsPtr;
    typedef std::unique_ptr<VisualizerParams> VisualizerParamsUniquePtr;


}


#endif
