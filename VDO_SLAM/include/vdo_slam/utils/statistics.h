#ifndef VDO_SLAM_STATISTICS_H
#define VDO_SLAM_STATISTICS_H

#include "vdo_slam/Params.h"
#include "vdo_slam/Scene.h"
#include "vdo_slam/utils/VdoUtils.h"
#include "vdo_slam/definitions.h"

#include <yaml-cpp/yaml.h>

namespace VDO_SLAM {


    const std::string RESULTS_DIR = std::string(__VDO_SLAM_DIR__) + "output_results/";

    struct Statistics {

        Statistics();
        virtual ~Statistics() = default;

    };


    class StatisticsManager {

        public:

            StatisticsManager(bool should_write_);
            StatisticsManager(const VdoParams& params_, bool should_write_);
            ~StatisticsManager() = default;

            void logScene(const Scene& scene);

            //for now this is just the gt odom we get from the bagfiles
            void logOdom(const Odometry& odom);

            void printStatistics();
            void writeStatistics();

        private:
            YAML::Node base_node_;

            //node to store system level information such as params, frequency etc
            YAML::Node system_node_;
            //node to store Scene/SceneObject information
            YAML::Node map_node_;
            //currently we will just use this for gt odom data so we can better do RMSE 
            YAML::Node additional_;

            bool should_write;

            //we save an extra params here so we dont have to write an decode function
            //when we want to print it
            VdoParams params;




    };

    typedef std::shared_ptr<StatisticsManager> StatisticsManagerPtr;
    typedef std::unique_ptr<StatisticsManager> StatisticsManagerUniquePtr;



}



#endif