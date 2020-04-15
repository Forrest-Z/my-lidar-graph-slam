
/* gnuplot_helper.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_IO_GNUPLOT_HELPER_HPP
#define MY_LIDAR_GRAPH_SLAM_IO_GNUPLOT_HELPER_HPP

#include <cstdio>
#include <memory>

#include "my_lidar_graph_slam/pose.hpp"
#include "my_lidar_graph_slam/mapping/pose_graph.hpp"

namespace MyLidarGraphSlam {
namespace IO {

/*
 * PipeDeleter struct is a custom deleter for std::unique_ptr
 */
struct PipeDeleter
{
    void operator()(FILE* ptr) const
    {
        if (ptr != nullptr)
            pclose(ptr);
    }
};

/*
 * GnuplotHelper class is for visualizing pose graphs to the Gnuplot window
 */
class GnuplotHelper
{
public:
    /* Default constructor */
    GnuplotHelper();
    /* Destructor */
    ~GnuplotHelper() = default;

    /* Copy constructor (disabled) */
    GnuplotHelper(const GnuplotHelper&) = delete;
    /* Copy assignment operator (disabled) */
    GnuplotHelper& operator=(const GnuplotHelper&) = delete;
    /* Move constructor (disabled) */
    GnuplotHelper(GnuplotHelper&&) = delete;
    /* Move assignment operator (disabled) */
    GnuplotHelper& operator=(GnuplotHelper&&) = delete;

    /* Draw the pose graph */
    void DrawPoseGraph(
        const std::shared_ptr<Mapping::PoseGraph>& poseGraph) const;

private:
    std::unique_ptr<FILE, PipeDeleter> mGnuplot;
};

} /* namespace IO */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_IO_GNUPLOT_HELPER_HPP */
