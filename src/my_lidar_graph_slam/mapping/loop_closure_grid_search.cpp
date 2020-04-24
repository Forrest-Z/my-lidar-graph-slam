
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
    const GridMapBuilderPtr& gridMapBuilder,
    const PoseGraphPtr& poseGraph,
    RobotPose2D<double>& relPose,
    int& startNodeIdx,
    int& endNodeIdx,
    Eigen::Matrix3d& estimatedCovMat)
{
    /* Retrieve the current robot pose and scan data */
    const auto& currentNode = poseGraph->LatestNode();
    const RobotPose2D<double>& currentPose = currentNode.Pose();
    const ScanPtr& currentScanData = currentNode.ScanData();
    const int currentNodeIdx = currentNode.Index();

    /* Find a local map and a pose graph node for loop closure */
    const auto loopClosureCandidates = this->mLoopClosureCandidate.Find(
        gridMapBuilder, poseGraph, currentPose);
    
    /* Do not perform loop closure if candidate not found */
    if (loopClosureCandidates.empty())
        return false;
    
    /* Find a corresponding pose of the current pose in the
     * loop-closure candidate local grid map */
    const int candidateMapIdx = loopClosureCandidates.front().first;
    const int candidateNodeIdx = loopClosureCandidates.front().second;

    const auto& candidateMapInfo = gridMapBuilder->LocalMapAt(candidateMapIdx);
    const GridMapType& candidateMap = candidateMapInfo.mMap;
    const auto& candidateNode = poseGraph->NodeAt(candidateNodeIdx);
    const RobotPose2D<double>& candidateNodePose = candidateNode.Pose();

    RobotPose2D<double> correspondingPose;
    Eigen::Matrix3d covMat;
    const bool loopFound = this->FindCorrespondingPose(
        candidateMap, currentScanData, currentPose,
        correspondingPose, covMat);
    
    /* Do not setup pose graph edge information if loop closure failed */
    if (!loopFound)
        return false;
    
    /* Setup pose graph edge information */
    /* Set the relative pose */
    relPose = InverseCompound(candidateNodePose, correspondingPose);
    /* Set the pose graph indices */
    startNodeIdx = candidateNodeIdx;
    endNodeIdx = currentNodeIdx;
    /* Set the estimated covariance matrix */
    estimatedCovMat = covMat;

    return true;
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
     * that minimizes the scan matching cost value */
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
