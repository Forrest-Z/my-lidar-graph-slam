
# CMakeLists.txt

target_sources(lidar_graph_slam PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/cost_function_greedy_endpoint.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/cost_function_square_error.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/grid_map_builder.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/lidar_graph_slam.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/lidar_graph_slam_backend.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/lidar_graph_slam_frontend.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/loop_detector_branch_bound.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/loop_detector_empty.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/loop_detector_grid_search.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/loop_detector_real_time_correlative.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/loop_searcher_nearest.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/motion_model_velocity.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/pose_graph_optimizer_lm.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/pose_graph.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/robust_loss_function.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/scan_accumulator.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/scan_interpolator.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/scan_matcher_branch_bound.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/scan_matcher_grid_search.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/scan_matcher_hill_climbing.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/scan_matcher_linear_solver.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/scan_matcher_real_time_correlative.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/score_function_pixel_accurate.cpp)
