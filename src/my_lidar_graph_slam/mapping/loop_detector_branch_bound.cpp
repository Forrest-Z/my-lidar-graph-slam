
/* loop_detector_branch_bound.cpp */

#include <cassert>
#include <deque>
#include <limits>

#include "my_lidar_graph_slam/io/map_saver.hpp"
#include "my_lidar_graph_slam/mapping/loop_detector_branch_bound.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/* Constructor */
LoopDetectorBranchBound::LoopDetectorBranchBound(
    const std::shared_ptr<ScanMatcherBranchBound>& scanMatcher,
    const double scoreThreshold) :
    mScanMatcher(scanMatcher),
    mScoreThreshold(scoreThreshold)
{
    assert(scoreThreshold > 0.0);
    assert(scoreThreshold <= 1.0);
}

/* Find a loop and return a loop constraint */
void LoopDetectorBranchBound::Detect(
    LoopDetectionQueryVector& loopDetectionQueries,
    LoopDetectionResultVector& loopDetectionResults)
{
    /* Clear the loop detection results */
    loopDetectionResults.clear();

    /* Do not perform loop detection if no query exists */
    if (loopDetectionQueries.empty())
        return;

    /* Perform loop detection for each query */
    for (auto& loopDetectionQuery : loopDetectionQueries) {
        /* Retrieve the information about each query */
        const auto& poseGraphNodes = loopDetectionQuery.mPoseGraphNodes;
        auto& localMapInfo = loopDetectionQuery.mLocalMapInfo;
        const auto& localMapNode = loopDetectionQuery.mLocalMapNode;

        /* Make sure that the node is inside the local grid map */
        assert(localMapNode.Index() >= localMapInfo.mPoseGraphNodeIdxMin &&
               localMapNode.Index() <= localMapInfo.mPoseGraphNodeIdxMax);

        /* Make sure that the grid map is in finished state */
        assert(localMapInfo.mFinished);

        /* Precompute the coarser local grid maps for efficiency */
        if (!localMapInfo.mPrecomputed) {
            /* Precompute multiple coarser grid maps */
            std::map<int, PrecomputedMapType> precompMaps =
                this->mScanMatcher->ComputeCoarserMaps(localMapInfo.mMap);
            /* Set the newly created grid map pyramid */
            localMapInfo.mPrecomputedMaps = std::move(precompMaps);
            /* Mark the current local map as precomputed */
            localMapInfo.mPrecomputed = true;
        }

        /* Perform loop detection for each node */
        for (const auto& poseGraphNode : poseGraphNodes) {
            /* Find the corresponding position of the node
             * inside the local grid map */
            RobotPose2D<double> correspondingPose;
            Eigen::Matrix3d covarianceMatrix;
            const bool loopDetected = this->FindCorrespondingPose(
                localMapInfo.mMap, localMapInfo.mPrecomputedMaps,
                poseGraphNode.ScanData(), poseGraphNode.Pose(),
                correspondingPose, covarianceMatrix);

            /* Do not build a new loop closing edge if loop not detected */
            if (!loopDetected)
                continue;

            /* Setup loop closing edge information */
            /* Relative pose of the edge */
            const RobotPose2D<double> relativePose =
                InverseCompound(localMapNode.Pose(), correspondingPose);
            /* Indices of the start and end node */
            const int startNodeIdx = localMapNode.Index();
            const int endNodeIdx = poseGraphNode.Index();

            /* Append to the loop detection results */
            loopDetectionResults.emplace_back(
                relativePose, localMapNode.Pose(),
                startNodeIdx, endNodeIdx, covarianceMatrix);
        }
    }

    return;
}

/* Find a corresponding pose of the current robot pose
 * from the local grid map */
bool LoopDetectorBranchBound::FindCorrespondingPose(
    const GridMapType& localMap,
    const std::map<int, PrecomputedMapType>& precompMaps,
    const Sensor::ScanDataPtr<double>& scanData,
    const RobotPose2D<double>& robotPose,
    RobotPose2D<double>& correspondingPose,
    Eigen::Matrix3d& estimatedCovMat) const
{
    /* Just call the scan matcher to find a corresponding pose */
    const auto matchingSummary = this->mScanMatcher->OptimizePose(
        localMap, precompMaps, scanData, robotPose, this->mScoreThreshold);

    /* Loop detection fails if the solution is not found */
    if (!matchingSummary.mPoseFound)
        return false;

    /* Return the result pose and the covariance in a world frame */
    correspondingPose = matchingSummary.mEstimatedPose;
    estimatedCovMat = matchingSummary.mEstimatedCovariance;

    return true;
}

} /* Mapping */
} /* namespace MyLidarGraphSlam */
