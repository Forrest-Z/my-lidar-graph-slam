
/* lidar_graph_slam_frontend.cpp */

#include "my_lidar_graph_slam/mapping/lidar_graph_slam_frontend.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/* Constructor */
LidarGraphSlamFrontend::LidarGraphSlamFrontend(
    const ScanInterpolatorPtr& scanInterpolator,
    const ScanMatcherPtr& scanMatcher,
    const RobotPose2D<double>& initialPose,
    const double updateThresholdTravelDist,
    const double updateThresholdAngle,
    const double updateThresholdTime,
    const int loopDetectionInterval) :
    mProcessCount(0),
    mScanInterpolator(scanInterpolator),
    mScanMatcher(scanMatcher),
    mInitialPose(initialPose),
    mLastOdomPose(0.0, 0.0, 0.0),
    mAccumulatedTravelDist(0.0),
    mAccumulatedAngle(0.0),
    mLastMapUpdateOdomPose(0.0, 0.0, 0.0),
    mLastMapUpdateTime(0.0),
    mUpdateThresholdTravelDist(updateThresholdTravelDist),
    mUpdateThresholdAngle(updateThresholdAngle),
    mUpdateThresholdTime(updateThresholdTime),
    mLoopDetectionInterval(loopDetectionInterval)
{
}

/* Process scan data and odometry information */
bool LidarGraphSlamFrontend::ProcessScan(
    LidarGraphSlam* const pParent,
    const Sensor::ScanDataPtr<double>& rawScanData,
    const RobotPose2D<double>& odomPose)
{
    /* Interpolate scan data if necessary */
    auto scanData = (this->mScanInterpolator != nullptr) ?
        this->mScanInterpolator->Interpolate(rawScanData) : rawScanData;

    /* Calculate the relative odometry pose */
    const RobotPose2D<double> relOdomPose = (this->mProcessCount == 0) ?
        RobotPose2D<double>(0.0, 0.0, 0.0) :
        InverseCompound(this->mLastOdomPose, odomPose);

    this->mLastOdomPose = odomPose;

    /* Accumulate the linear and angular distance */
    this->mAccumulatedTravelDist += Distance(relOdomPose);
    this->mAccumulatedAngle += std::fabs(relOdomPose.mTheta);

    /* Compute the elapsed time since the last map update */
    const double elapsedTime = (this->mProcessCount == 0) ?
        0.0 : scanData->TimeStamp() - this->mLastMapUpdateTime;

    /* Map is updated only when the robot moved more than the specified
     * distance or the specified time has elapsed since the last map update */
    const bool travelDistThreshold =
        this->mAccumulatedTravelDist >= this->mUpdateThresholdTravelDist;
    const bool angleThreshold =
        this->mAccumulatedAngle >= this->mUpdateThresholdAngle;
    const bool timeThreshold =
        elapsedTime >= this->mUpdateThresholdTime;
    const bool isFirstScan = this->mProcessCount == 0;
    const bool mapUpdateNeeded =
        travelDistThreshold || angleThreshold ||
        timeThreshold || isFirstScan;

    if (!mapUpdateNeeded)
        return false;

    /* Update the pose graph */
    if (this->mProcessCount == 0) {
        /* Set the initial pose for the first scan */
        RobotPose2D<double> estimatedPose = this->mInitialPose;
        /* Append the new pose graph node */
        pParent->AppendNode(estimatedPose, scanData);
    } else {
        /* Retrieve the latest pose and the latest map */
        RobotPose2D<double> latestPose;
        GridMapType latestMap;
        pParent->GetLatestPoseAndMap(latestPose, latestMap);

        const RobotPose2D<double> relPoseFromLastMapUpdate =
            InverseCompound(this->mLastMapUpdateOdomPose, odomPose);
        const RobotPose2D<double> initialPose =
            Compound(latestPose, relPoseFromLastMapUpdate);

        /* Perform scan matching against the grid map that contains
         * the latest scans and obtain the result */
        ScanMatcher::Summary scanMatchSummary;
        this->mScanMatcher->OptimizePose(
            latestMap, scanData, initialPose, scanMatchSummary);

        const RobotPose2D<double>& estimatedPose =
            scanMatchSummary.mEstimatedPose;
        const Eigen::Matrix3d& estimatedCovariance =
            scanMatchSummary.mEstimatedCovariance;

        /* Append a new pose graph node and odometry edge */
        pParent->AppendOdometryNodeAndEdge(
            estimatedPose, estimatedCovariance, scanData);
    }

    /* Integrate the scan data into the grid map and
     * check if the new local map is created */
    const bool localMapCreated = pParent->UpdateGridMap();

    /* Notify the worker thread for the SLAM backend
     * if the new local map is added to the entire grid map */
    if (this->mProcessCount > this->mLoopDetectionInterval &&
        this->mProcessCount % this->mLoopDetectionInterval == 0)
        pParent->NotifyBackend();

    /* Update miscellaneous parameters */
    this->mProcessCount += 1;
    this->mAccumulatedTravelDist = 0.0;
    this->mAccumulatedAngle = 0.0;
    this->mLastMapUpdateOdomPose = odomPose;
    this->mLastMapUpdateTime = scanData->TimeStamp();

    std::cerr << "Processing frame: " << this->mProcessCount << std::endl;

    return true;
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
