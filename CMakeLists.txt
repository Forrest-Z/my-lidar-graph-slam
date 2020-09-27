
# CMakeLists.txt

cmake_minimum_required(VERSION 3.8)

project(my_lidar_graph_slam CXX)

# Find libpng
find_package(PNG REQUIRED)

# Find Boost package
set(BOOST_USE_STATIC_LIBS ON)
find_package(Boost 1.65.1 REQUIRED)
include_directories(SYSTEM ${Boost_INCLUDE_DIRS})

# Find Eigen package
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

add_library(lidar_graph_slam)

add_subdirectory(include)
add_subdirectory(src)

target_include_directories(
    lidar_graph_slam PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(lidar_graph_slam PUBLIC m rt pthread dl stdc++fs)
target_link_libraries(lidar_graph_slam PUBLIC Eigen3::Eigen)
target_link_libraries(lidar_graph_slam PUBLIC PNG::PNG)
target_compile_options(lidar_graph_slam PUBLIC
    $<$<CXX_COMPILER_ID:GNU>:-pedantic -Wall -Wextra>
    $<$<CONFIG:Release>:-O3>
    $<$<AND:$<CXX_COMPILER_ID:GNU>,$<CONFIG:Debug>>:-O0 -ggdb>)
target_compile_features(lidar_graph_slam PUBLIC cxx_std_17)

add_executable(slam_launch
    ${PROJECT_SOURCE_DIR}/src/my_lidar_graph_slam/slam_launcher.cpp)
target_link_libraries(slam_launch lidar_graph_slam)
