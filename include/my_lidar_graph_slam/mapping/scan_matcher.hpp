
/* scan_matcher.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_SCAN_MATCHER_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_SCAN_MATCHER_HPP

#include <cmath>
#include <cstdlib>
#include <memory>

#include <Eigen/Core>

#include "my_lidar_graph_slam/pose.hpp"
#include "my_lidar_graph_slam/util.hpp"
#include "my_lidar_graph_slam/mapping/grid_map_builder.hpp"
#include "my_lidar_graph_slam/sensor/sensor_data.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/* Type definitions for convenience */
class ScanMatcher;
using ScanMatcherPtr = std::shared_ptr<ScanMatcher>;

class ScanMatcher
{
public:
    /*
     * Summary struct holds the details of the scan matching result
     */
    struct Summary
    {
        /* Normalized cost value */
        double              mNormalizedCost;
        /* Estimated robot pose */
        RobotPose2D<double> mEstimatedPose;
        /* Estimated pose covariance */
        Eigen::Matrix3d     mEstimatedCovariance;
    };

public:
    /* Type definitions */
    using GridMapType = GridMapBuilder::GridMapType;
    using PrecomputedMapType = GridMapBuilder::PrecomputedMapType;

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
        const GridMapType& gridMap,
        const Sensor::ScanDataPtr<double>& scanData,
        const RobotPose2D<double>& initialPose,
        Summary& resultSummary) = 0;

};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_SCAN_MATCHER_HPP */
