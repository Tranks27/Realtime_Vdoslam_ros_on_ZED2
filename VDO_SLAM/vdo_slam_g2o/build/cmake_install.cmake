# Install script for directory: /home/tranks/testing_ws/src/VDO_SLAM/vdo_slam_g2o

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libvdo_slam_g2o.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libvdo_slam_g2o.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libvdo_slam_g2o.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/tranks/testing_ws/src/VDO_SLAM/vdo_slam_g2o/build/libvdo_slam_g2o.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libvdo_slam_g2o.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libvdo_slam_g2o.so")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libvdo_slam_g2o.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/vdo_slam_g2o" TYPE DIRECTORY FILES "/home/tranks/testing_ws/src/VDO_SLAM/vdo_slam_g2o/include/vdo_slam_g2o/" FILES_MATCHING REGEX "/[^/]*\\.h$" REGEX "/[^/]*\\.hpp$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/vdo_slam_g2o/vdo_slam_g2o-config.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/vdo_slam_g2o/vdo_slam_g2o-config.cmake"
         "/home/tranks/testing_ws/src/VDO_SLAM/vdo_slam_g2o/build/CMakeFiles/Export/lib/cmake/vdo_slam_g2o/vdo_slam_g2o-config.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/vdo_slam_g2o/vdo_slam_g2o-config-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/vdo_slam_g2o/vdo_slam_g2o-config.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/vdo_slam_g2o" TYPE FILE FILES "/home/tranks/testing_ws/src/VDO_SLAM/vdo_slam_g2o/build/CMakeFiles/Export/lib/cmake/vdo_slam_g2o/vdo_slam_g2o-config.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/vdo_slam_g2o" TYPE FILE FILES "/home/tranks/testing_ws/src/VDO_SLAM/vdo_slam_g2o/build/CMakeFiles/Export/lib/cmake/vdo_slam_g2o/vdo_slam_g2o-config-release.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/vdo_slam_g2o" TYPE FILE FILES "/home/tranks/testing_ws/src/VDO_SLAM/vdo_slam_g2o/build/vdo_slam_g2o-config.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/vdo_slam_g2o" TYPE FILE FILES "/home/tranks/testing_ws/src/VDO_SLAM/vdo_slam_g2o/include/vdo_slam_g2o/vdo_slam_g2o_config.hpp")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/tranks/testing_ws/src/VDO_SLAM/vdo_slam_g2o/build/include/vdo_slam_g2o/core/cmake_install.cmake")
  include("/home/tranks/testing_ws/src/VDO_SLAM/vdo_slam_g2o/build/src/core/cmake_install.cmake")
  include("/home/tranks/testing_ws/src/VDO_SLAM/vdo_slam_g2o/build/include/vdo_slam_g2o/solvers/cmake_install.cmake")
  include("/home/tranks/testing_ws/src/VDO_SLAM/vdo_slam_g2o/build/src/solvers/cmake_install.cmake")
  include("/home/tranks/testing_ws/src/VDO_SLAM/vdo_slam_g2o/build/include/vdo_slam_g2o/stuff/cmake_install.cmake")
  include("/home/tranks/testing_ws/src/VDO_SLAM/vdo_slam_g2o/build/src/stuff/cmake_install.cmake")
  include("/home/tranks/testing_ws/src/VDO_SLAM/vdo_slam_g2o/build/include/vdo_slam_g2o/types/cmake_install.cmake")
  include("/home/tranks/testing_ws/src/VDO_SLAM/vdo_slam_g2o/build/src/types/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/tranks/testing_ws/src/VDO_SLAM/vdo_slam_g2o/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
