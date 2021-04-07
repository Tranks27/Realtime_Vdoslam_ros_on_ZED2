cmake_minimum_required(VERSION 3.0.2)
project(mono_depth_2)

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
  python_service_starter
  sensor_msgs
)
catkin_python_setup()

find_package(OpenCV)


add_service_files(
  FILES
  MonoDepth.srv
)


generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs std_msgs sensor_msgs)



include_directories(
  include
  ${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
	src/MonoDepthInterface.cpp
  )


target_link_libraries(${PROJECT_NAME}
		${catkin_LIBRARIES}
		${OpenCV_LIBS})



#add_message_files(
#    FILES
#)

catkin_package(
 INCLUDE_DIRS  include
 LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rospy std_msgs message_runtime roscpp
)



install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.hpp"
        PATTERN ".svn" EXCLUDE)

# install(DIRECTORY
#     #controller
#     #launch
#     #flow_net
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

catkin_install_python(PROGRAMS
    scripts/mono_depth_rospy.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
