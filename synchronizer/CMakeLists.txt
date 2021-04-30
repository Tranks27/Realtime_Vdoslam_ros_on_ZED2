cmake_minimum_required(VERSION 3.0.2)
project(synchronizer)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  message_filters
  message_generation
  roscpp
  sensor_msgs
  std_msgs
  actionlib_msgs
  mask_rcnn
  vision_msgs
)
find_package(OpenCV)







add_message_files(
  FILES
  Sync.msg
  SemanticObjectArray.msg
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
generate_messages(
  DEPENDENCIES
  sensor_msgs
  mask_rcnn
)


catkin_package(
  INCLUDE_DIRS #include
#  LIBRARIES synchronizer
  CATKIN_DEPENDS 
  cv_bridge 
  image_transport 
  message_filters 
  message_generation 
  roscpp 
  sensor_msgs 
  std_msgs 
  message_runtime
  mask_rcnn
  vision_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/synchronizer.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(${PROJECT_NAME}_node synchronizer_generate_messages_cpp)
#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_synchronizer.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
