
# CMakeLists.txt

target_sources(lidar_graph_slam PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/gnuplot_helper.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/map_saver.cpp)

add_subdirectory(carmen)
