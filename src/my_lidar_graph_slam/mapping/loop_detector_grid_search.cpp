
/* loop_detector_grid_search.cpp */

#include <cassert>
#include <limits>

#include "my_lidar_graph_slam/util.hpp"
#include "my_lidar_graph_slam/io/map_saver.hpp"
#include "my_lidar_graph_slam/mapping/loop_detector_grid_search.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/* Find a loop and return a loop constraint */
bool LoopDetectorGridSearch::Detect(
    LoopDetectionQueryVector& loopDetectionQueries,
    LoopDetectionResultVector& loopDetectionResults)
{
    /* Clear the loop detection results */
    loopDetectionResults.clear();

    /* Do not perform loop detection if no query exists */
    if (loopDetectionQueries.empty())
        return false;

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

    return !loopDetectionResults.empty();
}

/* Find a corresponding pose of the current robot pose
 * from the local grid map */
bool LoopDetectorGridSearch::FindCorrespondingPose(
    const GridMapType& gridMap,
    const ScanPtr& scanData,
    const RobotPose2D<double>& robotPose,
    RobotPose2D<double>& correspondingPose,
    Eigen::Matrix3d& estimatedCovMat) const
{
    /* Find the best pose from the search window
     * that maximizes the matching score value */
    const RobotPose2D<double> sensorPose =
        Compound(robotPose, scanData->RelativeSensorPose());
    RobotPose2D<double> bestSensorPose = sensorPose;
    double scoreMax = std::numeric_limits<double>::min();

    const double rx = this->mRangeX / 2.0;
    const double ry = this->mRangeY / 2.0;
    const double rt = this->mRangeTheta / 2.0;
    const double sx = this->mStepX;
    const double sy = this->mStepY;
    const double st = this->mStepTheta;

    /* Perform the grid search */
    for (double dy = -ry; dy <= ry; dy += sy) {
        for (double dx = -rx; dx <= rx; dx += sx) {
            for (double dt = -rt; dt <= rt; dt += st) {
                /* Calculate the cost value */
                const RobotPose2D<double> pose { sensorPose.mX + dx,
                                                 sensorPose.mY + dy,
                                                 sensorPose.mTheta + dt };
                ScoreFunction::Summary scoreSummary;
                this->mScoreFunc->Score(gridMap, scanData, pose, scoreSummary);

                /* Update the best pose and minimum cost value */
                if (scoreSummary.mMatchRate > this->mMatchRateThreshold &&
                    scoreSummary.mNormalizedScore > scoreMax) {
                    scoreMax = scoreSummary.mNormalizedScore;
                    bestSensorPose = pose;
                }
            }
        }
    }

    /* Loop detection fails if the matching score falls below the threshold */
    if (scoreMax < this->mScoreThreshold)
        return false;

    /* Estimate the covariance matrix */
    const Eigen::Matrix3d covMat =
        this->mCostFunc->ComputeCovariance(gridMap, scanData, bestSensorPose);

    /* Calculate the robot pose from the sensor pose */
    const RobotPose2D<double> bestPose =
        MoveBackward(bestSensorPose, scanData->RelativeSensorPose());

    /* Return the result */
    correspondingPose = bestPose;
    estimatedCovMat = covMat;

    return true;
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
