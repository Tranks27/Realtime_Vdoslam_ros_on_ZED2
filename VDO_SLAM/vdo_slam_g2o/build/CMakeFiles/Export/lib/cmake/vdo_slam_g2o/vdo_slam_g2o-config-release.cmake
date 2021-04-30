#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "vdo_slam_g2o" for configuration "Release"
set_property(TARGET vdo_slam_g2o APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vdo_slam_g2o PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libvdo_slam_g2o.so"
  IMPORTED_SONAME_RELEASE "libvdo_slam_g2o.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS vdo_slam_g2o )
list(APPEND _IMPORT_CHECK_FILES_FOR_vdo_slam_g2o "${_IMPORT_PREFIX}/lib/libvdo_slam_g2o.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
