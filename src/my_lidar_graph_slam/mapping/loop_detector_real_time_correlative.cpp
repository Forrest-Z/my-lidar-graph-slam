
/* loop_detector_real_time_correlative.cpp */

#include "my_lidar_graph_slam/mapping/loop_detector_real_time_correlative.hpp"

#include <algorithm>
#include <cassert>
#include <limits>
#include <numeric>

namespace MyLidarGraphSlam {
namespace Mapping {

/* Constructor */
LoopDetectorRealTimeCorrelative::LoopDetectorRealTimeCorrelative(
    const std::shared_ptr<ScanMatcherRealTimeCorrelative>& scanMatcher,
    const double scoreThreshold) :
    mScanMatcher(scanMatcher),
    mScoreThreshold(scoreThreshold)
{
    assert(scoreThreshold > 0.0);
    assert(scoreThreshold <= 1.0);
}

/* Find a loop and return a loop constraint */
void LoopDetectorRealTimeCorrelative::Detect(
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
        /* Retrieve the information for each query */
        const auto& poseGraphNodes = loopDetectionQuery.mPoseGraphNodes;
        auto& localMapInfo = loopDetectionQuery.mLocalMapInfo;
        const auto& localMapNode = loopDetectionQuery.mLocalMapNode;

        /* Make sure that the node is inside the local grid map */
        assert(localMapNode.Index() >= localMapInfo.mPoseGraphNodeIdxMin &&
               localMapNode.Index() <= localMapInfo.mPoseGraphNodeIdxMax);

        /* Make sure that the grid map is in finished state */
        assert(localMapInfo.mFinished);

        /* Precompute a low-resolution grid map */
        if (!localMapInfo.mPrecomputed) {
            /* Precompute a coarser grid map */
            PrecomputedMapType precompMap =
                this->mScanMatcher->ComputeCoarserMap(localMapInfo.mMap);
            /* Append the newly created grid map */
            localMapInfo.mPrecomputedMaps.emplace(0, std::move(precompMap));
            /* Mark the current local map as precomputed */
            localMapInfo.mPrecomputed = true;
        }

        /* The local grid map should have only one precomputed grid map */
        assert(localMapInfo.mPrecomputedMaps.size() == 1);

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
            /* Relative pose of the loop closing edge */
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
 * from a local grid map */
bool LoopDetectorRealTimeCorrelative::FindCorrespondingPose(
    const GridMapType& localMap,
    const std::map<int, PrecomputedMapType>& precompMaps,
    const Sensor::ScanDataPtr<double>& scanData,
    const RobotPose2D<double>& robotPose,
    RobotPose2D<double>& correspondingPose,
    Eigen::Matrix3d& estimatedCovMat) const
{
    /* Retrieve the precomputed low-resolution grid map */
    assert(precompMaps.size() == 1);
    const PrecomputedMapType& precompMap = precompMaps.at(0);

    /* Just call the scan matcher to find a corresponding pose */
    const auto matchingSummary = this->mScanMatcher->OptimizePose(
        localMap, precompMap, scanData, robotPose, this->mScoreThreshold);

    /* Loop detection fails if the score does not exceed the threshold */
    if (!matchingSummary.mPoseFound)
        return false;

    /* Return the result pose and the covariance in a world frame */
    correspondingPose = matchingSummary.mEstimatedPose;
    estimatedCovMat = matchingSummary.mEstimatedCovariance;

    return true;
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
