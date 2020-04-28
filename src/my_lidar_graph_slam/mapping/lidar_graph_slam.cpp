
/* lidar_graph_slam.cpp */

#include <cassert>
#include <cmath>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/LU>

#include "my_lidar_graph_slam/mapping/lidar_graph_slam.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/* Constructor */
LidarGraphSlam::LidarGraphSlam(
    const std::shared_ptr<GridMapBuilder>& gridMapBuilder,
    const ScanMatcherPtr& scanMatcher,
    const std::shared_ptr<PoseGraph>& poseGraph,
    const std::shared_ptr<PoseGraphOptimizer>& poseGraphOptimizer,
    const std::shared_ptr<LoopClosure>& loopClosure,
    int loopClosureInterval,
    const RobotPose2D<double>& initialPose,
    double updateThresholdTravelDist,
    double updateThresholdAngle,
    double updateThresholdTime) :
    mProcessCount(0),
    mGridMapBuilder(gridMapBuilder),
    mScanMatcher(scanMatcher),
    mPoseGraph(poseGraph),
    mPoseGraphOptimizer(poseGraphOptimizer),
    mLoopClosure(loopClosure),
    mLoopClosureInterval(loopClosureInterval),
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
}

/* Process scan data and odometry */
bool LidarGraphSlam::ProcessScan(
    const Sensor::ScanDataPtr<double>& scanData,
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
    
    /* Update the pose graph */
    if (this->mProcessCount == 0) {
        /* Set the initial pose for the first scan */
        RobotPose2D<double> estimatedPose = this->mInitialPose;
        /* Append the new pose graph node */
        this->mPoseGraph->AppendNode(estimatedPose, scanData);
    } else {
        const RobotPose2D<double> relPoseFromLastMapUpdate =
            InverseCompound(this->mLastMapUpdateOdomPose, odomPose);
        const RobotPose2D<double> initialPose =
            Compound(this->mPoseGraph->LatestNode().Pose(),
                     relPoseFromLastMapUpdate);
        
        /* Perform scan matching against the grid map that contains
         * the latest scans and obtain the result */
        ScanMatcher::Summary scanMatchSummary;
        this->mScanMatcher->OptimizePose(
            this->mGridMapBuilder->LatestMap(),
            scanData, initialPose, scanMatchSummary);
        
        const RobotPose2D<double>& estimatedPose =
            scanMatchSummary.mEstimatedPose;
        const Eigen::Matrix3d& estimatedCovariance =
            scanMatchSummary.mEstimatedCovariance;
        
        /* Append the new pose graph node */
        const RobotPose2D<double>& startNodePose =
            this->mPoseGraph->LatestNode().Pose();
        const int startNodeIdx =
            this->mPoseGraph->LatestNode().Index();
        const int endNodeIdx =
            this->mPoseGraph->AppendNode(estimatedPose, scanData);

        /* Two pose graph node indexes must be adjacent
         * since the edge represents the odometry constraint */
        assert(endNodeIdx == startNodeIdx + 1);
        
        /* Setup the pose graph edge parameters */
        /* Relative pose in the frame of the start node
         * Angular component must be normalized from -pi to pi */
        const RobotPose2D<double> edgeRelPose =
            NormalizeAngle(InverseCompound(startNodePose, estimatedPose));
        
        /* Covariance matrix must be rotated beforehand since the matrix
         * must represent the covariance in the node frame (not world frame) */
        Eigen::Matrix3d robotFrameCovMat;
        ConvertCovarianceFromWorldToRobot(
            startNodePose, estimatedCovariance, robotFrameCovMat);
        /* Calculate a information matrix by inverting a covariance matrix
         * obtained from the scan matching */
        const Eigen::Matrix3d edgeInfoMat = robotFrameCovMat.inverse();
        
        /* Append the new pose graph edge for odometric constraint */
        this->mPoseGraph->AppendEdge(startNodeIdx, endNodeIdx,
                                     edgeRelPose, edgeInfoMat);
    }

    /* Integrate the scan data into the grid map */
    this->mGridMapBuilder->AppendScan(this->mPoseGraph);
    
    /* Perform loop closure (pose graph optimization) */
    if (this->mProcessCount > this->mLoopClosureInterval &&
        this->mProcessCount % this->mLoopClosureInterval == 0) {
        /* Relative pose of the pose graph edge */
        RobotPose2D<double> relPose;
        /* Index of the pose graph node for starting pose */
        int startNodeIdx;
        /* Index of the pose graph node for ending pose */
        int endNodeIdx;
        /* Covariance matrix of the pose graph edge */
        Eigen::Matrix3d estimatedCovMat;

        /* Find a loop using a pose graph and local grid maps */
        const bool loopFound = this->mLoopClosure->FindLoop(
            this->mGridMapBuilder, this->mPoseGraph,
            relPose, startNodeIdx, endNodeIdx, estimatedCovMat);
        
        if (loopFound) {
            /* Setup the pose graph edge parameters */
            /* Relative pose must be normalized beforehand */
            relPose = NormalizeAngle(relPose);
            
            /* Covariance matrix must be rotated since the matrix
             * must represent the covariance in the node frame */
            const RobotPose2D<double>& startNodePose =
                this->mPoseGraph->NodeAt(startNodeIdx).Pose();
            Eigen::Matrix3d robotFrameCovMat;
            ConvertCovarianceFromWorldToRobot(
                startNodePose, estimatedCovMat, robotFrameCovMat);
            /* Calculate a information matrix by inverting a covariance matrix
             * obtained from the scan matching */
            const Eigen::Matrix3d edgeInfoMat = robotFrameCovMat.inverse();

            /* Append the new pose graph edge for loop closing constraint */
            this->mPoseGraph->AppendEdge(startNodeIdx, endNodeIdx,
                                         relPose, edgeInfoMat);
            
            /* Perform loop closure */
            this->mPoseGraphOptimizer->Optimize(this->mPoseGraph);

            /* Re-create the grid maps */
            this->mGridMapBuilder->AfterLoopClosure(this->mPoseGraph);
        }
    }

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
