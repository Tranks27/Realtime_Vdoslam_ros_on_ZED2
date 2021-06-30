cmake_minimum_required(VERSION 3.0.2)
project(mask_rcnn)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  visualization_msgs
  message_generation
  roscpp
  rospy
  std_msgs
  rostk_pyutils
  #python_service_starter
  sensor_msgs
  cv_bridge
  vision_msgs
)
catkin_python_setup()

add_message_files(
   FILES
   SemanticObject.msg
)

add_service_files(
  FILES
  MaskRcnnVdoSlam.srv
  MaskRcnnLabel.srv
  MaskRcnnLabelList.srv
)


generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs std_msgs sensor_msgs vision_msgs)



include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# add_library(${PROJECT_NAME}
# 	# src/MaskRcnnInterface.cpp
#   #src/SemanticTracker.cpp
#   #src/HungarianSolver.cpp
#   )


# target_link_libraries(${PROJECT_NAME}
# 		${catkin_LIBRARIES})


catkin_package(
 INCLUDE_DIRS  include
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rospy std_msgs message_runtime roscpp cv_bridge
)



# install(TARGETS ${PROJECT_NAME}
#         ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.hpp"
        PATTERN ".svn" EXCLUDE)


catkin_install_python(PROGRAMS
    scripts/mask_rcnn_rospy.py
  
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
