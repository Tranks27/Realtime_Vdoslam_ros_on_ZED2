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
  src/CameraInformation.cpp
)


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
  -lvdo_slam
  )

if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(ros_vdoslam_node
		  ${catkin_EXPORTED_LIBRARIES}
		  ${catkin_LIBRARIES}
		  ${OpenCV_LIBS}
      )
endif()
######## tranks addition

#####################

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







