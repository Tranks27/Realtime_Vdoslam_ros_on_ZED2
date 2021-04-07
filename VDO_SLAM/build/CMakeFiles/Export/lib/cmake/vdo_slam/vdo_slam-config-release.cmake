#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "vdo_slam" for configuration "Release"
set_property(TARGET vdo_slam APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(vdo_slam PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libvdo_slam.so"
  IMPORTED_SONAME_RELEASE "libvdo_slam.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS vdo_slam )
list(APPEND _IMPORT_CHECK_FILES_FOR_vdo_slam "${_IMPORT_PREFIX}/lib/libvdo_slam.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
