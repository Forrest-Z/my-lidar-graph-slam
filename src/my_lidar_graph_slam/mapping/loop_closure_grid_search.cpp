
/* loop_closure_grid_search.cpp */

#include <cassert>
#include <limits>

#include "my_lidar_graph_slam/util.hpp"
#include "my_lidar_graph_slam/io/map_saver.hpp"
#include "my_lidar_graph_slam/mapping/loop_closure_grid_search.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/* Find a loop and return a loop constraint */
bool LoopClosureGridSearch::FindLoop(
    LoopClosureCandidateInfoVector& loopClosureCandidates,
    LoopClosureResultVector& loopClosureResults)
{
    /* Clear the loop closure results */
    loopClosureResults.clear();

    /* Do not perform loop detection if no candidate exists */
    if (loopClosureCandidates.empty())
        return false;

    /* Perform loop detection for each candidate */
    for (const auto& loopClosureCandidate : loopClosureCandidates) {
        /* Retrieve the pose graph nodes, each of whose scan data is
         * matched against the local grid map to detect a loop */
        const auto& candidateNodes = loopClosureCandidate.mCandidateNodes;
        /* Retrieve the local grid map information */
        auto& localMapInfo = loopClosureCandidate.mLocalMapInfo;
        /* Retrieve the pose graph node inside the local grid map */
        const auto& localMapNode = loopClosureCandidate.mLocalMapNode;

        /* Make sure that the node is inside the local grid map */
        assert(localMapNode.Index() >= localMapInfo.mPoseGraphNodeIdxMin &&
               localMapNode.Index() <= localMapInfo.mPoseGraphNodeIdxMax);

        /* Perform loop detection for each candidate node */
        for (const auto& candidateNode : candidateNodes) {
            /* Find the corresponding position of the node
             * inside the local grid map */
            RobotPose2D<double> correspondingPose;
            Eigen::Matrix3d covarianceMatrix;
            const bool loopDetected = this->FindCorrespondingPose(
                localMapInfo.mMap,
                candidateNode.ScanData(), candidateNode.Pose(),
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
            const int endNodeIdx = candidateNode.Index();

            /* Append to the loop closure results */
            loopClosureResults.emplace_back(
                relativePose, startNodeIdx, endNodeIdx, covarianceMatrix);
        }
    }

    return !loopClosureResults.empty();
}

/* Find a corresponding pose of the current robot pose
 * from the loop-closure candidate local grid map */
bool LoopClosureGridSearch::FindCorrespondingPose(
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

    /* Loop closure fails if the matching score falls below the threshold */
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
