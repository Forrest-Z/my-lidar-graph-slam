
/* scan_matcher.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_SCAN_MATCHER_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_SCAN_MATCHER_HPP

#include <cmath>
#include <cstdlib>
#include <memory>

#include <Eigen/Core>

#include "my_lidar_graph_slam/pose.hpp"
#include "my_lidar_graph_slam/util.hpp"
#include "my_lidar_graph_slam/grid_map/grid_map_base.hpp"
#include "my_lidar_graph_slam/sensor/sensor_data.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

class ScanMatcher
{
public:
    /* Constructor */
    ScanMatcher() = default;
    /* Destructor */
    virtual ~ScanMatcher() = default;

    /* Copy constructor (disabled) */
    ScanMatcher(const ScanMatcher&) = delete;
    /* Copy assignment operator (disabled) */
    ScanMatcher& operator=(const ScanMatcher&) = delete;
    /* Move constructor (disabled) */
    ScanMatcher(ScanMatcher&&) = delete;
    /* Move assignment operator (disabled) */
    ScanMatcher& operator=(ScanMatcher&&) = delete;

    /* Optimize the robot pose by scan matching */
    virtual void OptimizePose(
        const GridMapBase<double>& gridMap,
        const Sensor::ScanDataPtr<double>& scanData,
        const RobotPose2D<double>& initialPose,
        RobotPose2D<double>& estimatedPose,
        double& normalizedCostValue) = 0;
    
    /* Calculate a covariance matrix */
    virtual void ComputeCovariance(
        const GridMapBase<double>& gridMap,
        const Sensor::ScanDataPtr<double>& scanData,
        const RobotPose2D<double>& robotPose,
        Eigen::Matrix3d& estimatedCovMat) = 0;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_SCAN_MATCHER_HPP */
