set(vdo_slam_g2o_INCLUDE_DIRS "/usr/local/include/vdo_slam_g2o")
set(vdo_slam_g2o_LIB_DIRS -lvdo_slam_g2o)


get_filename_component(VdoSlamG2O_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
include(CMakeFindDependencyMacro)

list(APPEND CMAKE_MODULE_PATH ${VdoSlamG2O_CMAKE_DIR})


# We should set those right?
#find_dependency(Thread REQUIRED)

#list(REMOVE_AT CMAKE_MODULE_PATH -1)
