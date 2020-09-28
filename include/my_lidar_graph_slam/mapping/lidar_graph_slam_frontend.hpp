
/* lidar_graph_slam_frontend.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_LIDAR_GRAPH_SLAM_FRONTEND_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_LIDAR_GRAPH_SLAM_FRONTEND_HPP

#include <memory>

#include "my_lidar_graph_slam/point.hpp"
#include "my_lidar_graph_slam/pose.hpp"
#include "my_lidar_graph_slam/util.hpp"
#include "my_lidar_graph_slam/mapping/lidar_graph_slam.hpp"
#include "my_lidar_graph_slam/mapping/scan_accumulator.hpp"
#include "my_lidar_graph_slam/mapping/scan_interpolator.hpp"
#include "my_lidar_graph_slam/mapping/scan_matcher.hpp"
#include "my_lidar_graph_slam/sensor/sensor_data.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

class LidarGraphSlamFrontend
{
public:
    /* Constructor */
    LidarGraphSlamFrontend(
        const ScanAccumulatorPtr& scanAccumulator,
        const ScanInterpolatorPtr& scanInterpolator,
        const ScanMatcherPtr& scanMatcher,
        const RobotPose2D<double>& initialPose,
        const double updateThresholdTravelDist,
        const double updateThresholdAngle,
        const double updateThresholdTime,
        const int loopDetectionInterval);
    /* Destructor */
    ~LidarGraphSlamFrontend() = default;

    /* Copy constructor (disabled) */
    LidarGraphSlamFrontend(const LidarGraphSlamFrontend&) = delete;
    /* Copy assignment operator (disabled) */
    LidarGraphSlamFrontend& operator=(const LidarGraphSlamFrontend&) = delete;
    /* Move constructor */
    LidarGraphSlamFrontend(LidarGraphSlamFrontend&&) = default;
    /* Move assignment operator */
    LidarGraphSlamFrontend& operator=(LidarGraphSlamFrontend&&) = default;

    /* Process scan data and odometry information */
    bool ProcessScan(LidarGraphSlam* const pParent,
                     const Sensor::ScanDataPtr<double>& rawScanData,
                     const RobotPose2D<double>& odomPose);

    /* Retrieve the total number of the processed input data */
    inline int ProcessCount() const { return this->mProcessCount; }

private:
    /* The total number of the processed input data */
    int                   mProcessCount;
    /* Scan accumulator */
    ScanAccumulatorPtr    mScanAccumulator;
    /* Scan interpolator */
    ScanInterpolatorPtr   mScanInterpolator;
    /* Scan matcher */
    ScanMatcherPtr        mScanMatcher;
    /* Initial pose */
    RobotPose2D<double>   mInitialPose;
    /* Last odometry pose */
    RobotPose2D<double>   mLastOdomPose;
    /* Accumulated travel distance since the last map update */
    double                mAccumulatedTravelDist;
    /* Accumulated angle since the last map update */
    double                mAccumulatedAngle;
    /* Odometry pose at the last map update */
    RobotPose2D<double>   mLastMapUpdateOdomPose;
    /* Time of the last map update */
    double                mLastMapUpdateTime;
    /* Map update threshold for accumulated travel distance */
    double                mUpdateThresholdTravelDist;
    /* Map update threshold for accumulated angle */
    double                mUpdateThresholdAngle;
    /* Map update threshold for the elapsed time since the last map update */
    double                mUpdateThresholdTime;
    /* Frame interval for performing loop detection */
    int                   mLoopDetectionInterval;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_LIDAR_GRAPH_SLAM_FRONTEND_HPP */
