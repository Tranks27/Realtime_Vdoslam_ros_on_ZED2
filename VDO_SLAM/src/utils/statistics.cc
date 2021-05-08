#include "vdo_slam/utils/statistics.h"
#include "vdo_slam/utils/seralization.h"
#include "vdo_slam/Macros.h"

#include <iostream>
#include <string>
#include <sstream> 


namespace VDO_SLAM {

    StatisticsManager::StatisticsManager(bool should_write_)
        :   should_write(should_write_) {

            VDO_INFO_MSG("Results dir: " << RESULTS_DIR << " and writing set to: " << should_write);

        }

    StatisticsManager::StatisticsManager(const VdoParams& params_, bool should_write_)
        :   should_write(should_write_) {

            VDO_INFO_MSG("Results dir: " << RESULTS_DIR << " and writing set to: " << should_write);
            system_node_["params"] = params_;
            params = params_;

        }

    void StatisticsManager::logScene(const Scene& scene) {
        map_node_["scenes"].push_back(scene);
    }

    void StatisticsManager::logOdom(const Odometry& odom) {
        additional_["odom_gt"].push_back(odom);
    }


    void StatisticsManager::printStatistics() {
        if (!system_node_["params"].IsDefined()) {
            VDO_ERROR_MSG("Cannot print params becuase they are not set");
            return;
        }

        std::stringstream out;
        out << "Statistics\n";

        out << "fx: " << params.fx << "\n";
        out << "fy: " << params.fy << "\n";
        out << "cx: " << params.cx << "\n";

        std::string out_string = out.str();

        VDO_INFO_MSG(out_string);
    }

    
    void StatisticsManager::writeStatistics() {
        if (should_write) {
            std::string system_file = RESULTS_DIR + "system.yaml";
            std::string map_file = RESULTS_DIR + "map.yaml";
            std::string additional_file = RESULTS_DIR + "add.yaml";

            std::ofstream fout(system_file);
            fout << system_node_;
            VDO_INFO_MSG("Written data to " << system_file);

            std::ofstream fout_map(map_file);
            fout_map << map_node_;
            VDO_INFO_MSG("Written data to " << map_file);

            std::ofstream fout_add(additional_file);
            fout_add << additional_;
            VDO_INFO_MSG("Written data to " << additional_file);

        }

    }

}


