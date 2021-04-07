target_sources(vdo_slam_g2o PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/csparse_extension.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/solver_csparse.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/csparse_helper.cpp"
)