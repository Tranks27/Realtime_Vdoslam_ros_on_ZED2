cmake_minimum_required(VERSION 3.0.2)
project(dumb_image_processor)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  
)
find_package(OpenCV)

catkin_package(
 INCLUDE_DIRS #include
#  LIBRARIES dumb_image_processor
 CATKIN_DEPENDS roscpp 
                std_msgs
                cv_bridge
                image_transport
                sensor_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

#build
add_executable(${PROJECT_NAME}_node src/main.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(${PROJECT_NAME}_node_2 src/main2.cpp)
target_link_libraries(${PROJECT_NAME}_node_2 ${catkin_LIBRARIES} ${OpenCV_LIBS})
