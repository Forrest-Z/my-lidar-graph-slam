
/* loop_detector_grid_search.cpp */

#include <cassert>
#include <limits>

#include "my_lidar_graph_slam/util.hpp"
#include "my_lidar_graph_slam/io/map_saver.hpp"
#include "my_lidar_graph_slam/mapping/loop_detector_grid_search.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/* Constructor */
LoopDetectorGridSearch::LoopDetectorGridSearch(
    const std::shared_ptr<ScanMatcherGridSearch>& scanMatcher,
    const double scoreThreshold,
    const double knownRateThreshold) :
    mScanMatcher(scanMatcher),
    mScoreThreshold(scoreThreshold),
    mKnownRateThreshold(knownRateThreshold)
{
    Assert(scoreThreshold > 0.0 && scoreThreshold <= 1.0);
    Assert(knownRateThreshold > 0.0 && knownRateThreshold <= 1.0);
}

/* Find a loop and return a loop constraint */
void LoopDetectorGridSearch::Detect(
    LoopDetectionQueryVector& loopDetectionQueries,
    LoopDetectionResultVector& loopDetectionResults)
{
    /* Clear the loop detection results */
    loopDetectionResults.clear();

    /* Do not perform loop detection if no query exists */
    if (loopDetectionQueries.empty())
        return;

    /* Perform loop detection for each query */
    for (const auto& loopDetectionQuery : loopDetectionQueries) {
        /* Retrieve the scan nodes, each of whose scan data is
         * matched against the local grid map to detect a loop */
        const auto& scanNodes = loopDetectionQuery.mScanNodes;
        /* Retrieve the local grid map information */
        auto& localMap = loopDetectionQuery.mLocalMap;
        /* Retrieve the local map node */
        const auto& localMapNode = loopDetectionQuery.mLocalMapNode;

        /* Check the local map Id */
        Assert(localMap.mId == localMapNode.mLocalMapId);

        /* Perform loop detection for each scan node */
        for (const auto& scanNode : scanNodes) {
            /* Compute the scan node pose in a map-local coordinate frame */
            const RobotPose2D<double> mapLocalScanPose =
                InverseCompound(localMapNode.mGlobalPose, scanNode.mGlobalPose);
            /* Find the corresponding position of the scan node
             * inside the local grid map */
            RobotPose2D<double> correspondingPose;
            Eigen::Matrix3d covarianceMatrix;
            const bool loopDetected = this->FindCorrespondingPose(
                localMap.mMap, scanNode.mScanData, mapLocalScanPose,
                correspondingPose, covarianceMatrix);

            /* Do not build a new loop closing edge if loop not detected */
            if (!loopDetected)
                continue;

            /* Append to the loop detection results */
            /* Relative pose and covariance matrix in a map-local coordinate
             * frame is already obtained */
            loopDetectionResults.emplace_back(
                correspondingPose, localMapNode.mGlobalPose,
                localMapNode.mLocalMapId, scanNode.mNodeId, covarianceMatrix);
        }
    }

    return;
}

/* Find a corresponding pose of the current robot pose
 * from the local grid map */
bool LoopDetectorGridSearch::FindCorrespondingPose(
    const GridMapType& gridMap,
    const Sensor::ScanDataPtr<double>& scanData,
    const RobotPose2D<double>& mapLocalScanPose,
    RobotPose2D<double>& correspondingPose,
    Eigen::Matrix3d& estimatedCovMat) const
{
    /* Just call the exhaustive grid search scan matcher */
    const auto matchingSummary = this->mScanMatcher->OptimizePose(
        gridMap, scanData, mapLocalScanPose,
        this->mScoreThreshold, this->mKnownRateThreshold);

    /* Return the result */
    correspondingPose = matchingSummary.mEstimatedPose;
    estimatedCovMat = matchingSummary.mEstimatedCovariance;

    /* Loop detection fails if the matching score falls below the threshold */
    if (!matchingSummary.mPoseFound)
        return false;

    return true;
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
