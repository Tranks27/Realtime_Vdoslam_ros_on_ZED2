#ifndef VDO_SLAM_MAP_OBJECT_H
#define VDO_SLAM_MAP_OBJECT_H

#include <memory>



namespace VDO_SLAM {

    class Map;

     /**
     * @brief Represents an object that can be reconstructed from the map. 
     * 
     */
    class MapObject {

        public:
            /**
             * @brief Virtual function overload called when the parent child 
             * object needs updating from the some data structure from the map.  
             * 
             * @param map const Map* a reference to the map and underlying data structures. 
             * A const reference would be better but we will use this 'this' pointer instead.
             * @return true 
             * @return false 
             */
            virtual bool update_from_map(const Map* map) = 0;

    };

    typedef std::shared_ptr<MapObject> MapObjectPtr;
}


#endif