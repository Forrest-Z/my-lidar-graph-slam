
# CMakeLists.txt

target_sources(lidar_graph_slam PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/cost_function_greedy_endpoint.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/cost_function_square_error.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/grid_map_builder.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/lidar_graph_slam.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/loop_closure_branch_bound.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/loop_closure_candidate_nearest.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/loop_closure_empty.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/loop_closure_grid_search.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/motion_model_velocity.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/pose_graph_optimizer_lm.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/pose_graph.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/scan_interpolator.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/scan_matcher_hill_climbing.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/scan_matcher_linear_solver.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/scan_matcher_real_time_correlative.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/score_function_pixel_accurate.cpp)
