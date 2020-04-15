
# CMakeLists.txt

target_sources(lidar_graph_slam PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/grid_map_builder.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/lidar_graph_slam.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/pose_graph.cpp)
