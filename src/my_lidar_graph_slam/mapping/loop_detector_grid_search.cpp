
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
    const double scoreThreshold) :
    mScanMatcher(scanMatcher),
    mScoreThreshold(scoreThreshold)
{
    assert(scoreThreshold > 0.0);
    assert(scoreThreshold <= 1.0);
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
        /* Retrieve the pose graph nodes, each of whose scan data is
         * matched against the local grid map to detect a loop */
        const auto& poseGraphNodes = loopDetectionQuery.mPoseGraphNodes;
        /* Retrieve the local grid map information */
        auto& localMapInfo = loopDetectionQuery.mLocalMapInfo;
        /* Retrieve the pose graph node inside the local grid map */
        const auto& localMapNode = loopDetectionQuery.mLocalMapNode;

        /* Make sure that the node is inside the local grid map */
        assert(localMapNode.Index() >= localMapInfo.mPoseGraphNodeIdxMin &&
               localMapNode.Index() <= localMapInfo.mPoseGraphNodeIdxMax);

        /* Perform loop detection for each node */
        for (const auto& poseGraphNode : poseGraphNodes) {
            /* Find the corresponding position of the node
             * inside the local grid map */
            RobotPose2D<double> correspondingPose;
            Eigen::Matrix3d covarianceMatrix;
            const bool loopDetected = this->FindCorrespondingPose(
                localMapInfo.mMap,
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
bool LoopDetectorGridSearch::FindCorrespondingPose(
    const GridMapType& gridMap,
    const Sensor::ScanDataPtr<double>& scanData,
    const RobotPose2D<double>& robotPose,
    RobotPose2D<double>& correspondingPose,
    Eigen::Matrix3d& estimatedCovMat) const
{
    /* Just call the exhaustive grid search scan matcher */
    const auto matchingSummary = this->mScanMatcher->OptimizePose(
        gridMap, scanData, robotPose, this->mScoreThreshold);

    /* Loop detection fails if the matching score falls below the threshold */
    if (!matchingSummary.mPoseFound)
        return false;

    /* Return the result */
    correspondingPose = matchingSummary.mEstimatedPose;
    estimatedCovMat = matchingSummary.mEstimatedCovariance;

    return true;
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
