
# CMakeLists.txt

target_sources(lidar_graph_slam PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/my_lidar_graph_slam.cpp)

add_subdirectory(io)
add_subdirectory(mapping)
