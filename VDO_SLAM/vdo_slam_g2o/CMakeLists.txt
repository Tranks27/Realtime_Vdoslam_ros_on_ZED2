cmake_minimum_required(VERSION 3.10)
SET(CMAKE_LEGACY_CYGWIN_WIN32 0)

PROJECT(vdo_slam_g2o VERSION 1.0 LANGUAGES CXX)

set(LIB_NAME ${PROJECT_NAME})


SET(g2o_C_FLAGS)
SET(g2o_CXX_FLAGS)

# default built type
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE(STATUS "BUILD TYPE:" ${CMAKE_BUILD_TYPE})
set(PACKAGE_INCLUDE_INSTALL_LOCATION ${CMAKE_INSTALL_PREFIX}/include/${PACKAGE_NAME})
message("PACKAGE_INCLUDE_INSTALL_LOCATION: ${PACKAGE_INCLUDE_INSTALL_LOCATION}")
message("Current src dir: ${CMAKE_CURRENT_SOURCE_DIR}")
list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_SOURCE_DIR}/cmake)



SET (include_LIB_TYPE SHARED)

# There seems to be an issue with MSVC8
# see http://eigen.tuxfamily.org/bz/show_bug.cgi?id=83
if(MSVC90)
  add_definitions(-DEIGEN_DONT_ALIGN_STATICALLY=1)
  message(STATUS "Disabling memory alignment for MSVC8")
endif(MSVC90)

# Set the output directory for the build executables and libraries
IF(WIN32)
  SET(include_LIBRARY_OUTPUT_DIRECTORY  ${PROJECT_BINARY_DIR}/bin CACHE PATH "Target for the libraries")
ELSE(WIN32)
  SET(include_LIBRARY_OUTPUT_DIRECTORY  ${PROJECT_BINARY_DIR}/lib CACHE PATH "Target for the libraries")
ENDIF(WIN32)
# SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${g2o_LIBRARY_OUTPUT_DIRECTORY})
# SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${g2o_LIBRARY_OUTPUT_DIRECTORY})
# SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${g2o_RUNTIME_OUTPUT_DIRECTORY})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)
set(CMAKE_SRC_BINARY_DIRECTORY ${CMAKE_BINARY_DIR}/src)
set(g2o_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# Set search directory for looking for our custom CMake scripts to
# look for SuiteSparse, QGLViewer, and Eigen3.
MESSAGE("G20 source dir " ${g2o_SOURCE_DIR})
LIST(APPEND CMAKE_MODULE_PATH ${g2o_SOURCE_DIR}/cmake_modules)

# Detect OS and define macros appropriately
IF(UNIX)
  ADD_DEFINITIONS(-DUNIX)
  REMOVE_DEFINITIONS(-D_MSC_VER)
  MESSAGE(STATUS "Compiling on Unix")
ENDIF(UNIX)

# Eigen library parallelise itself, though, presumably due to performance issues
# OPENMP is experimental. We experienced some slowdown with it

FIND_PACKAGE(OpenMP)
SET(G2O_USE_OPENMP OFF CACHE BOOL "Build include with OpenMP support (EXPERIMENTAL)")
IF(OPENMP_FOUND AND G2O_USE_OPENMP)
  SET (G2O_OPENMP 1)
  SET(g2o_C_FLAGS "${g2o_C_FLAGS} ${OpenMP_C_FLAGS}")
  SET(g2o_CXX_FLAGS "${g2o_CXX_FLAGS} -DEIGEN_DONT_PARALLELIZE ${OpenMP_CXX_FLAGS}")
  MESSAGE(STATUS "Compiling with OpenMP support")
ENDIF(OPENMP_FOUND AND G2O_USE_OPENMP)

# Compiler specific options for gcc
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=native")
SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -march=native")

# activate warnings !!!
SET(g2o_C_FLAGS "${g2o_C_FLAGS} -Wall -W")
SET(g2o_CXX_FLAGS "${g2o_CXX_FLAGS} -Wall -W")

# specifying compiler flags
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${g2o_CXX_FLAGS}")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${g2o_C_FLAGS}")

# Find Eigen3
SET(EIGEN3_INCLUDE_DIR ${include_EIGEN3_INCLUDE})
FIND_PACKAGE(Eigen3 3.1.0 REQUIRED)
IF(EIGEN3_FOUND)
  SET(include_EIGEN3_INCLUDE ${EIGEN3_INCLUDE_DIR} CACHE PATH "Directory of Eigen3")
ELSE(EIGEN3_FOUND)
  SET(include_EIGEN3_INCLUDE "" CACHE PATH "Directory of Eigen3")
ENDIF(EIGEN3_FOUND)

# Set the package install location
set(PACKAGE_INCLUDE_INSTALL_LOCATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME})
message("PACKAGE_INCLUDE_INSTALL_LOCATION: ${PACKAGE_INCLUDE_INSTALL_LOCATION}")


# Generate vdoslam_g2o_config.h
SET(include_CXX_COMPILER "${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER}")

find_package(CSparse REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)


# configure_file(cmake/${PROJECT_NAME}.hpp.in ${PROJECT_NAME}_config.hpp)
# set(VDO_SLAM_G2O_CONFIG_FILE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_config.hpp)

configure_file(cmake/${PROJECT_NAME}.hpp.in ${CMAKE_CURRENT_LIST_DIR}/include/${PROJECT_NAME}/${PROJECT_NAME}_config.hpp)
set(VDO_SLAM_G2O_CONFIG_FILE ${CMAKE_CURRENT_LIST_DIR}/include/${PROJECT_NAME}/${PROJECT_NAME}_config.hpp)

# include(CMakePackageConfigHelpers)
# configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}-config.cmake.in
#         ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
#         INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
# )

include(CMakePackageConfigHelpers)
set(INSTALL_CONFIGDIR lib/cmake/${PROJECT_NAME})
configure_package_config_file(
  ${CMAKE_CURRENT_LIST_DIR}/cmake/${PROJECT_NAME}-config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
  INSTALL_DESTINATION ${INSTALL_CONFIGDIR}
)


include_directories(${CSPARSE_INCLUDE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

# Include the subdirectories
add_library(${LIB_NAME} ${include_LIB_TYPE} "")

add_subdirectory(include/vdo_slam_g2o/core)
add_subdirectory(src/core)

add_subdirectory(include/vdo_slam_g2o/solvers)
add_subdirectory(src/solvers)

add_subdirectory(include/vdo_slam_g2o/stuff)
add_subdirectory(src/stuff)

add_subdirectory(include/vdo_slam_g2o/types)
add_subdirectory(src/types)


target_link_libraries(${LIB_NAME} 
  ${CSPARSE_LIBRARY}
  ${OPENGL_LIBRARIES})

target_compile_options(${LIB_NAME}  PRIVATE -Wno-psabi)

# Set include directories for build and install
target_include_directories(${LIB_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${include_EIGEN3_INCLUDE}
)

install(TARGETS ${LIB_NAME} 
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

#Install the config file to usr include space
install(FILES
        ${VDO_SLAM_G2O_CONFIG_FILE}
        DESTINATION include/${PROJECT_NAME}
)
