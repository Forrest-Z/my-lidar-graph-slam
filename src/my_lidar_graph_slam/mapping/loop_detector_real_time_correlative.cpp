
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
        const auto& scanNodes = loopDetectionQuery.mScanNodes;
        auto& localMap = loopDetectionQuery.mLocalMap;
        const auto& localMapNode = loopDetectionQuery.mLocalMapNode;

        /* Check the local map Id */
        Assert(localMap.mId == localMapNode.mLocalMapId);
        /* Make sure that the local grid map is in finished state */
        Assert(localMap.mFinished);

        /* Precompute a low-resolution grid map */
        if (!localMap.mPrecomputed) {
            /* Precompute a coarser grid map */
            ConstMapType precompMap =
                this->mScanMatcher->ComputeCoarserMap(localMap.mMap);
            /* Append the newly created grid map */
            localMap.mPrecomputedMaps.emplace(0, std::move(precompMap));
            /* Mark the current local map as precomputed */
            localMap.mPrecomputed = true;
        }

        /* The local grid map should have only one precomputed grid map */
        Assert(localMap.mPrecomputedMaps.size() == 1);

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
 * from a local grid map */
bool LoopDetectorRealTimeCorrelative::FindCorrespondingPose(
    const GridMapType& localMap,
    const std::map<int, ConstMapType>& precompMaps,
    const Sensor::ScanDataPtr<double>& scanData,
    const RobotPose2D<double>& mapLocalScanPose,
    RobotPose2D<double>& correspondingPose,
    Eigen::Matrix3d& estimatedCovMat) const
{
    /* Retrieve the precomputed low-resolution grid map */
    Assert(precompMaps.size() == 1);
    const ConstMapType& precompMap = precompMaps.at(0);

    /* Just call the scan matcher to find a corresponding pose */
    const auto matchingSummary = this->mScanMatcher->OptimizePose(
        localMap, precompMap, scanData,
        mapLocalScanPose, this->mScoreThreshold);

    /* Loop detection fails if the score does not exceed the threshold */
    if (!matchingSummary.mPoseFound)
        return false;

    /* Return the result pose and the covariance in a map-local frame */
    correspondingPose = matchingSummary.mEstimatedPose;
    estimatedCovMat = matchingSummary.mEstimatedCovariance;

    return true;
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
