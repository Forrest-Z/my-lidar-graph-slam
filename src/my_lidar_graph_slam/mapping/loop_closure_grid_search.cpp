
/* loop_closure_grid_search.cpp */

#include <cassert>
#include <limits>

#include "my_lidar_graph_slam/util.hpp"
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
    int candidateMapIdx;
    int candidateNodeIdx;
    bool candidateFound = this->FindLoopClosureCandidates(
        gridMapBuilder, poseGraph, currentPose,
        candidateMapIdx, candidateNodeIdx);
    
    /* Do not perform loop closure if candidate not found */
    if (!candidateFound)
        return false;
    
    /* Find a corresponding pose of the current pose in the
     * loop-closure candidate local grid map */
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

/* Find a local map and a pose graph node as loop closure candidates */
bool LoopClosureGridSearch::FindLoopClosureCandidates(
    const GridMapBuilderPtr& gridMapBuilder,
    const PoseGraphPtr& poseGraph,
    const RobotPose2D<double>& robotPose,
    int& candidateMapIdx,
    int& candidateNodeIdx) const
{
    const int numOfMaps = static_cast<int>(gridMapBuilder->LocalMaps().size());
    const int numOfNodes = static_cast<int>(poseGraph->Nodes().size());

    assert(numOfMaps > 0);
    assert(numOfNodes > 0);

    /* Find the index of the local grid map and the pose graph node
     * that may contain loop-closure point */
    double nodeDistMinSq = std::pow(this->mPoseGraphNodeDistMax, 2.0);
    bool candidateFound = false;

    candidateMapIdx = numOfMaps;
    candidateNodeIdx = numOfNodes;

    /* Exclude the latest local grid map */
    for (int mapIdx = 0; mapIdx < numOfMaps - 1; ++mapIdx) {
        /* Retrieve the local grid map */
        const auto& localMapInfo = gridMapBuilder->LocalMapAt(mapIdx);
        const int nodeIdxMin = localMapInfo.mPoseGraphNodeIdxMin;
        const int nodeIdxMax = localMapInfo.mPoseGraphNodeIdxMax;

        for (int nodeIdx = nodeIdxMin; nodeIdx <= nodeIdxMax; ++nodeIdx) {
            /* Retrieve the pose graph node */
            const auto& node = poseGraph->NodeAt(nodeIdx);
            const RobotPose2D<double>& pose = node.Pose();

            /* Calculate the distance between the pose graph node and
             * the current pose */
            const double nodeDistSq = SquaredDistance(pose, robotPose);

            /* Update the candidate map index and node index */
            if (nodeDistSq < nodeDistMinSq) {
                candidateFound = true;
                nodeDistMinSq = nodeDistSq;
                candidateMapIdx = mapIdx;
                candidateNodeIdx = nodeIdx;
            }
        }
    }

    return candidateFound;
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
    double costMin = std::numeric_limits<double>::max();
    
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
                const double cost =
                    this->mCostFunc->Cost(gridMap, scanData, pose);

                /* Update the best pose and minimum cost value */
                if (cost < costMin) {
                    costMin = cost;
                    bestSensorPose = pose;
                }
            }
        }
    }

    /* Calculate the robot pose from the sensor pose */
    const RobotPose2D<double> bestPose =
        MoveBackward(bestSensorPose, scanData->RelativeSensorPose());

    /* Perform scan matching at the best pose */
    double normalizedCostValue;
    this->mScanMatcher->OptimizePose(
        gridMap, scanData, bestPose,
        correspondingPose, normalizedCostValue);
    
    /* Loop closure fails if the cost value exceeds the threshold */
    if (normalizedCostValue >= this->mCostThreshold)
        return false;
    
    /* Estimate the covariance matrix */
    this->mScanMatcher->ComputeCovariance(
        gridMap, scanData, correspondingPose, estimatedCovMat);

    return true;
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
