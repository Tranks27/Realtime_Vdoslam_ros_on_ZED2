set(vdo_slam_INCLUDE_DIRS "/usr/local/include/vdo_slam")
set(vdo_slam_LIB_DIRS -lvdo_slam)


get_filename_component(VdoSlam_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
include(CMakeFindDependencyMacro)

list(APPEND CMAKE_MODULE_PATH ${VdoSlam_CMAKE_DIR})


# We should set those right?
#find_dependency(Thread REQUIRED)

#list(REMOVE_AT CMAKE_MODULE_PATH -1)
