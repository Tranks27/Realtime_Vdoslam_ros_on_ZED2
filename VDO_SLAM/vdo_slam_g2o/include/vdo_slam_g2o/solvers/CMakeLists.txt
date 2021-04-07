target_sources(vdo_slam_g2o PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/g2o_csparse_extension_api.h"
    "${CMAKE_CURRENT_LIST_DIR}/g2o_csparse_api.h"
    "${CMAKE_CURRENT_LIST_DIR}/csparse_extension.h"
    "${CMAKE_CURRENT_LIST_DIR}/csparse_helper.h"
    "${CMAKE_CURRENT_LIST_DIR}/linear_solver_csparse.h"
    "${CMAKE_CURRENT_LIST_DIR}/linear_solver_dense.h"
)