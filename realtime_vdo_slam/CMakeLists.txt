cmake_minimum_required(VERSION 3.10)
project(realtime_vdo_slam)



## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	rosbag
	#rosbag_storage
	#gmsl_frame_msg
	roscpp
	tf2_ros
	tf2
	tf
	tf2_sensor_msgs
	image_transport
	flow_net
	mask_rcnn
	# mono_depth_2
	# python_service_starter
	geometry_msgs
	tf2_geometry_msgs
	visualization_msgs
	message_generation
	vision_msgs
	std_msgs
	cv_bridge
	nav_msgs
	# midas_ros
	sensor_msgs
	message_filters
	nodelet
)



find_package(OpenCV)
find_package(vdo_slam REQUIRED)
# find_package(GTSAM REQUIRED)
# find_package(minisam REQUIRED)
# set(minisam_INCLUDE_DIR /usr/local/include/minisam/)



# include(VerifyGtsamConfig)
# option(ROS_VDO_SLAM_VERIFY_GTSAM_CONFIG "Check that GTSAM was compiled with the right options" OFF)
# if (ROS_VDO_SLAM_VERIFY_GTSAM_CONFIG)
#     verify_gtsam_config()
# endif()
## Setup include directories
#include_directories(${catkin_INCLUDE_DIRS},
#  include/${PROJECT_NAME}
#  include)



## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   VdoSlamScene.msg
   VdoSlamMap.msg
   VdoSceneObject.msg
   VdoInput.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs std_msgs sensor_msgs mask_rcnn vision_msgs)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need


set(PROJECT_INCLUDE_DIRS
	include
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${vdo_slam_INCLUDE_DIRS}
	# ${CMAKE_CURRENT_SOURCE_DIR}
	# ${GTSAM_INCLUDE_DIR})
	# ${minisam_INCLUDE_DIR}
)

include_directories(
	${PROJECT_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
	# src/preprocessing/ImagePreprocessing.cpp
	# src/preprocessing/ImagePreprocessingRGB.cpp
	# src/preprocessing/ImagePreprocessingRGBD.cpp
	src/RosVdoSlam.cpp
	#src/RosScene.cpp
	src/CameraInformation.cpp
	src/utils/RosUtils.cpp
	src/tracking/HungarianSolver.cpp
	src/tracking/SemanticTracker.cpp
	src/visualizer/RosVisualizer.cpp
	src/visualizer/RosAsyncManager.cpp
	# src/data_provider/DataProviderInterface.cpp
	# src/data_provider/RosBagDataProvider.cpp
	# src/scene_graph/SceneGraphBuilder.cpp
	# src/scene_graph/SceneGraph.cpp
	# src/scene_graph/SceneGraphOptimizer.cpp
	)

catkin_package(
    LIBRARIES ${PROJECT_NAME}
  	INCLUDE_DIRS ${PROJECT_INCLUDE_DIRS}
	CATKIN_DEPENDS
	roscpp
	rosbag
	image_transport
	# python_service_starter
	std_msgs
	tf2_ros
	tf2
	tf
	tf2_sensor_msgs
	nav_msgs
	sensor_msgs
	geometry_msgs
	tf2_geometry_msgs
	cv_bridge
	flow_net
	mask_rcnn
	# mono_depth_2
	# midas_ros
	message_runtime
	message_filters
	nodelet
	# DEPENDS
	# minisam
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# target_link_libraries(${PROJECT_NAME}_scene_graph
# 	${catkin_LIBRARIES}
# 	-lminisam)


target_link_libraries(${PROJECT_NAME}
		${catkin_LIBRARIES}
		${OpenCV_LIBS}
		${vdo_slam_LIB_DIRS}
		# gtsam
    	# gtsam_unstable
		# minisam
		)


add_executable(ros_vdoslam_node
		src/RosVdoSlamNode.cpp)

target_link_libraries(ros_vdoslam_node
		${catkin_LIBRARIES}
		${OpenCV_LIBS}
		${PROJECT_NAME}
        -lvdo_slam
		)

if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(ros_vdoslam_node
		  ${catkin_EXPORTED_LIBRARIES}
		  ${catkin_LIBRARIES}
		  ${OpenCV_LIBS})
endif()


# add_executable(image_preprocessing_node
# 		src/ImagePreprocessingNode.cpp
# 		)



# target_link_libraries(image_preprocessing_node
# 		${catkin_LIBRARIES}
# 		${OpenCV_LIBS}
# 		${PROJECT_NAME}
#         -lvdo_slam
# 		)

# if(catkin_EXPORTED_LIBRARIES)
#   add_dependencies(image_preprocessing_node
# 		  ${catkin_EXPORTED_LIBRARIES}
# 		  ${catkin_LIBRARIES}
# 		  ${OpenCV_LIBS})
# endif()


#### Vdo Slam Scene Graph Node
# add_executable(vdo_slam_scene_graph_node
# 		src/VdoSlamSceneGraphNode.cpp
		
# 		)

# # target_include_directories(vdo_slam_scene_graph_node PUBLIC
# # 	${PROJECT_INCLUDE_DIRS}
# # 	${minisam_INCLUDE_DIR})


# target_link_libraries(vdo_slam_scene_graph_node 
# 		${catkin_LIBRARIES}
# 		${OpenCV_LIBS}
# 		${PROJECT_NAME}
# 		)

# if(catkin_EXPORTED_LIBRARIES)
#   add_dependencies(vdo_slam_scene_graph_node
#   		  ${PROJECT_NAME}
# 		  ${catkin_EXPORTED_LIBRARIES}
# 		  ${catkin_LIBRARIES}
# 		  ${OpenCV_LIBS})
# endif()



## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
#add_executable(${PROJECT_NAME}_node src/camera_read.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ros_vdoslam_node image_preprocessing_node_rgb
#    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

install(TARGETS ros_vdoslam_node 
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install(TARGETS ros_vdoslam_node vdo_slam_scene_graph_node
#    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_camera_read.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
