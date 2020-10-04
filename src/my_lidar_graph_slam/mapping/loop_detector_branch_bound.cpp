
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
        const auto& scanNodes = loopDetectionQuery.mScanNodes;
        auto& localMap = loopDetectionQuery.mLocalMap;
        const auto& localMapNode = loopDetectionQuery.mLocalMapNode;

        /* Check the local map Id */
        Assert(localMap.mId == localMapNode.mLocalMapId);
        /* Make sure that the grid map is in finished state */
        Assert(localMap.mFinished);

        /* Precompute the coarser local grid maps for efficiency */
        if (!localMap.mPrecomputed) {
            /* Precompute multiple coarser grid maps */
            std::map<int, ConstMapType> precompMaps =
                this->mScanMatcher->ComputeCoarserMaps(localMap.mMap);
            /* Set the newly created grid map pyramid */
            localMap.mPrecomputedMaps = std::move(precompMaps);
            /* Mark the current local map as precomputed */
            localMap.mPrecomputed = true;
        }

        /* Perform loop detection for each node */
        for (const auto& scanNode : scanNodes) {
            /* Compute the scan node pose in a map-local coordinate frame */
            const RobotPose2D<double> mapLocalScanPose =
                InverseCompound(localMapNode.mGlobalPose, scanNode.mGlobalPose);
            /* Find the corresponding position of the scan node
             * inside the local grid map */
            RobotPose2D<double> correspondingPose;
            Eigen::Matrix3d covarianceMatrix;
            const bool loopDetected = this->FindCorrespondingPose(
                localMap.mMap, localMap.mPrecomputedMaps,
                scanNode.mScanData, mapLocalScanPose,
                correspondingPose, covarianceMatrix);

            /* Do not build a new loop closing edge if loop not detected */
            if (!loopDetected)
                continue;

            /* Append to the loop detection results */
            loopDetectionResults.emplace_back(
                correspondingPose, localMapNode.mGlobalPose,
                localMapNode.mLocalMapId, scanNode.mNodeId, covarianceMatrix);
        }
    }

    return;
}

/* Find a corresponding pose of the current robot pose
 * from the local grid map */
bool LoopDetectorBranchBound::FindCorrespondingPose(
    const GridMapType& localMap,
    const std::map<int, ConstMapType>& precompMaps,
    const Sensor::ScanDataPtr<double>& scanData,
    const RobotPose2D<double>& mapLocalScanPose,
    RobotPose2D<double>& correspondingPose,
    Eigen::Matrix3d& estimatedCovMat) const
{
    /* Just call the scan matcher to find a corresponding pose */
    const auto matchingSummary = this->mScanMatcher->OptimizePose(
        localMap, precompMaps, scanData,
        mapLocalScanPose, this->mScoreThreshold);

    /* Loop detection fails if the solution is not found */
    if (!matchingSummary.mPoseFound)
        return false;

    /* Return the result pose and the covariance in a map-local frame */
    correspondingPose = matchingSummary.mEstimatedPose;
    estimatedCovMat = matchingSummary.mEstimatedCovariance;

    return true;
}

} /* Mapping */
} /* namespace MyLidarGraphSlam */
