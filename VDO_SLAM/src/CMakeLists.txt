target_sources(vdo_slam PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/Converter.cc"
    "${CMAKE_CURRENT_LIST_DIR}/Frame.cc"
    "${CMAKE_CURRENT_LIST_DIR}/Optimizer.cc"
    "${CMAKE_CURRENT_LIST_DIR}/ORBextractor.cc"
    "${CMAKE_CURRENT_LIST_DIR}/System.cc"
    "${CMAKE_CURRENT_LIST_DIR}/Scene.cc"
    "${CMAKE_CURRENT_LIST_DIR}/Tracking.cc"
)