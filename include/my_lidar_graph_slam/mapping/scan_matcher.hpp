
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

/*
 * ScanMatchingQuery struct holds the necessary information for
 * executing a scan matching, such as an initial robot pose in a world frame,
 * a grid map, and a scan data to be matched against a grid map
 */
struct ScanMatchingQuery final
{
    /* Constructor */
    ScanMatchingQuery(GridMapType&& gridMap,
                      const Sensor::ScanDataPtr<double>& scanData,
                      const RobotPose2D<double>& initialPose) :
        mGridMap(std::move(gridMap)),
        mScanData(scanData),
        mInitialPose(initialPose) { }

    /* Destructor */
    ~ScanMatchingQuery() = default;

    /* Grid map */
    const GridMapType                 mGridMap;
    /* Scan data */
    const Sensor::ScanDataPtr<double> mScanData;
    /* Initial robot pose (in a world frame) */
    const RobotPose2D<double>         mInitialPose;
};

/*
 * ScanMatchingSummary struct holds the details of the scan matching result
 */
struct ScanMatchingSummary final
{
    /* Constructor */
    ScanMatchingSummary(const bool poseFound,
                        const double normalizedCost,
                        const RobotPose2D<double>& initialPose,
                        const RobotPose2D<double>& estimatedPose,
                        const Eigen::Matrix3d& estimatedCovariance) :
        mPoseFound(poseFound),
        mNormalizedCost(normalizedCost),
        mInitialPose(initialPose),
        mEstimatedPose(estimatedPose),
        mEstimatedCovariance(estimatedCovariance) { }

    /* Destructor */
    ~ScanMatchingSummary() = default;

    /* Flag to indicate whether the appropriate pose is found */
    const bool                mPoseFound;
    /* Normalized cost value */
    const double              mNormalizedCost;
    /* Initial robot pose in a world frame */
    const RobotPose2D<double> mInitialPose;
    /* Estimated pose in a world frame */
    const RobotPose2D<double> mEstimatedPose;
    /* Estimated pose covariance matrix in a world frame */
    const Eigen::Matrix3d     mEstimatedCovariance;
};

/* Type definitions for convenience */
class ScanMatcher;
using ScanMatcherPtr = std::shared_ptr<ScanMatcher>;

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
    virtual ScanMatchingSummary OptimizePose(
        const ScanMatchingQuery& queryInfo) = 0;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_SCAN_MATCHER_HPP */
