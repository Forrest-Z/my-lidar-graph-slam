
/* scan_matcher.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_SCAN_MATCHER_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_SCAN_MATCHER_HPP

#include "my_lidar_graph_slam/pose.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

template <typename T, typename U>
class ScanMatcher
{
public:
    /* Type definitions */
    using GridMapType = T;
    using ScanType = U;

    /* Constructor */
    ScanMatcher() = default;
    /* Destructor */
    virtual ~ScanMatcher() = default;

    /* Optimize the robot pose by scan matching */
    virtual void OptimizePose(const GridMapType& gridMap,
                              const ScanType& scanData,
                              const RobotPose2D<double>& initialPose,
                              RobotPose2D<double>& estimatedPose) = 0;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_SCAN_MATCHER_HPP */
