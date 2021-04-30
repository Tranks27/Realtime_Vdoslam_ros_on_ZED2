cmake_minimum_required(VERSION 3.10)
project(vdo_slam VERSION 1.0)

set(PACKAGE_NAME ${PROJECT_NAME})



IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()


option(DEBUG "Builds with Debug flag on" OFF)

if (DEBUG)
        message("Setting VDO_DEBUG to True.")
        add_definitions(-DVDO_DEBUG)
else()
   message("VDO_DEBUG flag set to False.")
endif()


MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

# ======  start c++ compile  ======

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)


find_package(OpenCV 3.2 REQUIRED)
# if(NOT OpenCV_FOUND)
#    find_package(OpenCV 2.4.3 QUIET)
#    if(NOT OpenCV_FOUND)
#       message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
#    endif()
# endif()

find_package(Eigen3 3.1.0 REQUIRED)
find_package(CSparse REQUIRED)
find_package(vdo_slam_g2o REQUIRED)

include_directories(
   include
   include/${PROJECT_NAME}
   # ${PROJECT_SOURCE_DIR}
   # ${PROJECT_SOURCE_DIR}/include
   ${EIGEN3_INCLUDE_DIR}
   ${CSPARSE_INCLUDE_DIR}
)

# set(PUBLIC_HEADER_FILES
#     include/vdo_slam.hpp
#     include/${PROJECT_NAME}/Converter.h
#     include/${PROJECT_NAME}/Frame.h
#     include/${PROJECT_NAME}/Optimizer.h
#     include/${PROJECT_NAME}/ORBextractor.h
#     include/${PROJECT_NAME}/System.h
#     include/${PROJECT_NAME}/Scene.h
#     include/${PROJECT_NAME}/Tracking.h
#     include/${PROJECT_NAME}/Params.h
#     include/${PROJECT_NAME}/Macros.h
#     include/${PROJECT_NAME}/utils/Types.h
#     include/${PROJECT_NAME}/utils/VdoUtils.h

#     include/${PROJECT_NAME}/map/Map.h
#     include/${PROJECT_NAME}/map/MapObject.h

#     include/${PROJECT_NAME}/cvplot/color.h
#     include/${PROJECT_NAME}/cvplot/cvplot.h
#     include/${PROJECT_NAME}/cvplot/figure.h
#     include/${PROJECT_NAME}/cvplot/highgui.h
#     include/${PROJECT_NAME}/cvplot/window.h
#     include/${PROJECT_NAME}/cvplot/internal.h
# )

# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

include(CMakePackageConfigHelpers)
set(INSTALL_CONFIGDIR lib/cmake/${PROJECT_NAME})
configure_package_config_file(
  ${CMAKE_CURRENT_LIST_DIR}/cmake/${PROJECT_NAME}-config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
  INSTALL_DESTINATION ${INSTALL_CONFIGDIR}
)



# add_library(${PROJECT_NAME} SHARED
#    src/utils/Types.cc
#    src/utils/VdoUtils.cc
#    src/System.cc
#    src/Tracking.cc
#    src/ORBextractor.cc
#    src/Converter.cc
#    src/Optimizer.cc
#    src/Frame.cc
#    src/Scene.cc

#    src/map/Map.cc 
#    # src/map/MapObject.cc

#    src/cvplot/color.cc
#    src/cvplot/figure.cc
#    src/cvplot/highgui.cc
#    src/cvplot/window.cc
#    ${PUBLIC_HEADER_FILES}
# )

add_library(${PROJECT_NAME} SHARED "")

# add_subdirectory(include/vdo_slam/cvplot)
# add_subdirectory(src/cvplot)

add_subdirectory(include/vdo_slam/map)
add_subdirectory(src/map)

add_subdirectory(include/vdo_slam/utils)
add_subdirectory(src/utils)

add_subdirectory(include/vdo_slam/visualizer)
add_subdirectory(src/visualizer)

add_subdirectory(include/vdo_slam)
add_subdirectory(src)

message("vdo slam g2o lib ${vdo_slam_g2o_LIB_DIRS}")

target_link_libraries(${PROJECT_NAME} PUBLIC
   ${OpenCV_LIBS}
   ${EIGEN3_LIBS}
   ${CSPARSE_LIBRARY}
   ${vdo_slam_g2o_LIB_DIRS}
)

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
   ${OpenCV_INCLUDE_DIRS}
   ${vdo_slam_g2o_INCLUDE_DIRS}
   ${include_EIGEN3_INCLUDE}
   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
   $<INSTALL_INTERFACE:include>
)

# Configure target warnings (PRIVATE doesn't pass on options to targets linking to target)
target_compile_options(${PROJECT_NAME} PRIVATE -Wno-psabi)


add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})


# Install library as an export target
install(TARGETS ${PACKAGE_NAME}
        EXPORT ${PROJECT_NAME}-export
        PUBLIC_HEADER DESTINATION include/${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )

# Install header files
install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME}
        FILES_MATCHING
        PATTERN "*.h"
        PATTERN "*.hpp")




# Install export library to generate cmake
install(EXPORT ${PROJECT_NAME}-export
   #NAMESPACE ${PACKAGE_NAME}::
   FILE ${PROJECT_NAME}-config.cmake
   DESTINATION lib/cmake/${PROJECT_NAME}
   )

# Install package config file
install(FILES
   ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
   DESTINATION lib/cmake/${PROJECT_NAME}
   )

# # Install Header Files
# install(FILES ${PUBLIC_HEADER_FILES} DESTINATION include/${PROJECT_NAME})
# install(FILES include/${PROJECT_NAME}.hpp DESTINATION include/${PACKAGE_NAME})



# Build example

#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/example)

# add_executable(vdo_slam_example
#   example/vdo_slam.cc
# )
# target_link_libraries(vdo_slam_example ${PROJECT_NAME}
# ${vdo_slam_g2o_LIB_DIRS})


