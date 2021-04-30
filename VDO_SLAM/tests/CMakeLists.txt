cmake_minimum_required(VERSION 3.10)
project(vdo_slam_tests)


set(CMAKE_CXX_STANDARD 14)

find_package(vdo_slam REQUIRED)
find_package(vdo_slam_g2o REQUIRED)



add_executable(test_colors testColors.cc)
# Include directories
target_include_directories(test_colors PUBLIC)


target_link_libraries(test_colors
	${vdo_slam_LIB_DIRS})	



