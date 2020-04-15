
/* lidar_graph_slam.cpp */

#include <cassert>
#include <cmath>
#include <iostream>

#include <Eigen/Core>

#include "my_lidar_graph_slam/mapping/lidar_graph_slam.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/* Constructor */
LidarGraphSlam::LidarGraphSlam(
    std::unique_ptr<ScanMatcherType>&& scanMatcher,
    double mapResolution,
    int patchSize,
    int numOfScansForLatestMap,
    double travelDistThresholdForLocalMap,
    const RobotPose2D<double>& initialPose,
    double updateThresholdTravelDist,
    double updateThresholdAngle,
    double updateThresholdTime,
    double usableRangeMin,
    double usableRangeMax) :
    mProcessCount(0),
    mGridMapBuilder(nullptr),
    mPoseGraph(nullptr),
    mScanMatcher(std::move(scanMatcher)),
    mInitialPose(initialPose),
    mLastOdomPose(0.0, 0.0, 0.0),
    mAccumulatedTravelDist(0.0),
    mAccumulatedAngle(0.0),
    mLastMapUpdateOdomPose(0.0, 0.0, 0.0),
    mLastMapUpdateTime(0.0),
    mUpdateThresholdTravelDist(updateThresholdTravelDist),
    mUpdateThresholdAngle(updateThresholdAngle),
    mUpdateThresholdTime(updateThresholdTime)
{
    assert(mapResolution > 0.0);
    assert(updateThresholdTravelDist > 0.0);
    assert(updateThresholdAngle > 0.0);
    assert(updateThresholdTime > 0.0);
    assert(usableRangeMin >= 0.0);
    assert(usableRangeMax >= 0.0);

    /* Construct the grid map */
    this->mGridMapBuilder = std::make_shared<GridMapBuilder>(
        mapResolution, patchSize, numOfScansForLatestMap,
        travelDistThresholdForLocalMap, usableRangeMin, usableRangeMax);
    
    /* Construct the pose graph */
    this->mPoseGraph = std::make_shared<PoseGraph>();
}

/* Process scan data and odometry */
bool LidarGraphSlam::ProcessScan(const ScanType& scanData,
                                 const RobotPose2D<double>& odomPose)
{
    /* Calculate the relative odometry pose */
    const RobotPose2D<double> relOdomPose = (this->mProcessCount == 0) ?
        RobotPose2D<double>(0.0, 0.0, 0.0) :
        InverseCompound(this->mLastOdomPose, odomPose);
    
    this->mLastOdomPose = odomPose;

    /* Accumulate the linear and angular distance */
    this->mAccumulatedTravelDist +=
        std::sqrt(relOdomPose.mX * relOdomPose.mX +
                  relOdomPose.mY * relOdomPose.mY);
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
    const bool timeThreshold = elapsedTime >= this->mUpdateThresholdTime;
    const bool isFirstScan = this->mProcessCount == 0;
    const bool mapUpdateNeeded =
        travelDistThreshold || angleThreshold || timeThreshold || isFirstScan;
    
    if (!mapUpdateNeeded)
        return false;
    
    /* Calculate the refined pose */
    RobotPose2D<double> estimatedPose;

    if (this->mProcessCount == 0) {
        /* Set the initial pose for the first scan */
        estimatedPose = this->mInitialPose;
    } else {
        /* Perform scan matching against the grid map
         * that contains latest scans */
        const RobotPose2D<double> relPoseFromLastMapUpdate =
            InverseCompound(this->mLastMapUpdateOdomPose, odomPose);
        const RobotPose2D<double> initialPose =
            Compound(this->mPoseGraph->LatestNode().Pose(),
                     relPoseFromLastMapUpdate);
        this->mScanMatcher->OptimizePose(
            this->mGridMapBuilder->LatestMap(),
            scanData, initialPose, estimatedPose);
    }

    /* Update the pose graph */
    if (this->mProcessCount == 0) {
        /* Append the new pose graph node */
        this->mPoseGraph->AppendNode(estimatedPose, scanData);
    } else {
        /* Append the new pose graph node */
        const int startNodeIdx =
            this->mPoseGraph->LatestNode().Index();
        const RobotPose2D<double>& startNodePose =
            this->mPoseGraph->LatestNode().Pose();
        const int endNodeIdx =
            this->mPoseGraph->AppendNode(estimatedPose, scanData);

        /* Two pose graph node indexes must be adjacent
         * since the edge represents the odometry constraint */
        assert(endNodeIdx == startNodeIdx + 1);
        
        /* Setup the pose graph edge parameters */
        const RobotPose2D<double> edgeRelPose =
            InverseCompound(startNodePose, estimatedPose);
        /* TODO: Calculate information matrix properly using the
         * covariance matrices obtained from odometry and scan matching */
        const Eigen::Matrix3d edgeInfoMat =
            Eigen::Matrix3d::Identity();
        
        /* Append the new pose graph edge */
        this->mPoseGraph->AppendEdge(startNodeIdx, endNodeIdx,
                                     edgeRelPose, edgeInfoMat);
    }

    /* Integrate the scan data into the grid map */
    this->mGridMapBuilder->AppendScan(this->mPoseGraph);
    
    /* TODO: Perform loop closure (pose graph optimization) */

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
