cmake_minimum_required(VERSION 3.10)
project(my_realtime_vdo_slam)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  flow_net
  geometry_msgs
  image_transport
  mask_rcnn
  message_generation
  midas_ros
  mono_depth_2
  nav_msgs
  python_service_starter
  roscpp
  sensor_msgs
  std_msgs
  tf
  tf2
  tf2_geometry_msgs
  tf2_ros
  tf2_sensor_msgs
  vision_msgs
  visualization_msgs
  nodelet
  message_filters
)

find_package(OpenCV)
find_package(vdo_slam REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   VdoSlamScene.msg
   VdoSlamMap.msg
   VdoSceneObject.msg
   VdoInput.msg
)

generate_messages(
  DEPENDENCIES 
    actionlib_msgs 
    geometry_msgs 
    std_msgs 
    sensor_msgs 
    mask_rcnn 
    vision_msgs
)


set(PROJECT_INCLUDE_DIRS
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${vdo_slam_INCLUDE_DIRS}
  )

include_directories(
  ${PROJECT_INCLUDE_DIRS}
)  


catkin_package(
  INCLUDE_DIRS ${PROJECT_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME} 
  CATKIN_DEPENDS 
    roscpp
    image_transport 
    python_service_starter 
    std_msgs 
    tf 
    tf2 
    tf2_geometry_msgs 
    tf2_ros 
    tf2_sensor_msgs 
    nav_msgs 
    sensor_msgs 
    geometry_msgs 
    cv_bridge 
    flow_net 
    mask_rcnn 
    mono_depth_2 
    midas_ros 
    message_runtime 
    nodelet
    message_filters
  # DEPENDS system_lib
)

add_library(${PROJECT_NAME} SHARED 
  src/RosVdoSlam.cpp
  src/utils/RosUtils.cpp
  src/visualizer/RosVisualizer.cpp
	src/visualizer/RosAsyncManager.cpp
  src/tracking/HungarianSolver.cpp
	src/tracking/SemanticTracker.cpp
)
# if(${vdo_slam_LIB_DIRS})
#   message(status "I am LIB_DIRS")
# else()
#   message(status "I am not LIB_DIRS")
# endif()

target_link_libraries(${PROJECT_NAME} PUBLIC
		${catkin_LIBRARIES}
		${OpenCV_LIBS}
		${vdo_slam_LIBRARIES}
)
# if(catkin_EXPORTED_LIBRARIES)
#   add_dependencies(${PROJECT_NAME}
# 		  ${catkin_EXPORTED_LIBRARIES}
# 		  ${catkin_LIBRARIES}
# 		  ${OpenCV_LIBS}
#       ${vdo_slam_LIB_DIRS})
# endif()
# target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
#     ${catkin_INCLUDE_DIRS}
#     ${OpenCV_INCLUDE_DIRS}
#     ${vdo_slam_INCLUDE_DIRS}
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:include>
# )


add_executable(ros_vdoslam_node 
  src/ros_vdoslam_node.cpp
)

target_link_libraries(ros_vdoslam_node 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PROJECT_NAME}
  )

if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(ros_vdoslam_node
		  ${catkin_EXPORTED_LIBRARIES}
		  ${catkin_LIBRARIES}
		  ${OpenCV_LIBS}
      )
endif()

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
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_my_realtime_vdo_slam.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


install(TARGETS ros_vdoslam_node 
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})







