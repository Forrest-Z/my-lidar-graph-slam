
/* loop_closure_branch_bound.cpp */

#include <cassert>
#include <deque>
#include <limits>

#include "my_lidar_graph_slam/io/map_saver.hpp"
#include "my_lidar_graph_slam/mapping/loop_closure_branch_bound.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/* Find a loop and return a loop constraint */
bool LoopClosureBranchBound::FindLoop(
    GridMapBuilderPtr& gridMapBuilder,
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
    const auto& candidateNode = poseGraph->NodeAt(candidateNodeIdx);
    const RobotPose2D<double>& candidateNodePose = candidateNode.Pose();

    /* The local grid map used for loop closure must be finished */
    assert(candidateMapInfo.mFinished);

    /* Precompute local grid maps for efficient loop closure */
    if (!candidateMapInfo.mPrecomputed)
        gridMapBuilder->PrecomputeGridMaps(
            candidateMapIdx, this->mNodeHeightMax);

    RobotPose2D<double> correspondingPose;
    Eigen::Matrix3d covMat;
    const bool loopFound = this->FindCorrespondingPose(
        candidateMapInfo, currentScanData, currentPose,
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
bool LoopClosureBranchBound::FindCorrespondingPose(
    const GridMapBuilder::LocalMapInfo& localMapInfo,
    const ScanPtr& scanData,
    const RobotPose2D<double>& robotPose,
    RobotPose2D<double>& correspondingPose,
    Eigen::Matrix3d& estimatedCovMat) const
{
    /* Retrieve the local grid map */
    const GridMapType& localMap = localMapInfo.mMap;
    /* Retrieve the precomputed grid maps */
    const auto& precompMaps = localMapInfo.mPrecomputedMaps;

    /* Find the best pose from the search window
     * that maximizes the matching score value */
    const RobotPose2D<double> sensorPose =
        Compound(robotPose, scanData->RelativeSensorPose());
    RobotPose2D<double> bestSensorPose = sensorPose;
    double scoreMax = this->mScoreThreshold;
    bool solutionFound = false;

    /* Determine the search window step */
    const double mapResolution = localMap.Resolution();
    const double deltaX = mapResolution;
    const double deltaY = mapResolution;

    const auto maxRangeIt = std::max_element(
        scanData->Ranges().cbegin(), scanData->Ranges().cend());
    const double maxRange = std::min(*maxRangeIt, this->mScanRangeMax);
    const double deltaTheta = std::acos(
        1.0 - 0.5 * (mapResolution / maxRange) * (mapResolution / maxRange));

    /* Determine the search window */
    const double rangeX = this->mRangeX / 2.0;
    const double rangeY = this->mRangeY / 2.0;
    const double rangeTheta = this->mRangeTheta / 2.0;
    const int winX = static_cast<int>(std::ceil(rangeX / deltaX));
    const int winY = static_cast<int>(std::ceil(rangeY / deltaY));
    const int winTheta = static_cast<int>(std::ceil(rangeTheta / deltaTheta));

    std::stack<Node> nodeStack;

    /* Initialize a stack with nodes covering the entire search window */
    const int winSizeMax = static_cast<int>(std::pow(2, this->mNodeHeightMax));

    for (int x = -winX; x <= winX; x += winSizeMax)
        for (int y = -winY; y <= winY; y += winSizeMax)
            for (int t = -winTheta; t <= winTheta; ++t)
                nodeStack.emplace(x, y, t, this->mNodeHeightMax);

    /* Find the best solution that maximizes the score value
     * using Branch-and-Bound method */
    while (!nodeStack.empty()) {
        /* Retrieve the node */
        const Node& currentNode = nodeStack.top();
        /* Compute the corresponding node pose */
        const RobotPose2D<double> nodePose {
            sensorPose.mX + currentNode.mX * deltaX,
            sensorPose.mY + currentNode.mY * deltaY,
            sensorPose.mTheta + currentNode.mTheta * deltaTheta };
        /* Calculate node score */
        ScoreFunction::Summary resultSummary;
        this->mScoreFunc->Score(precompMaps.at(currentNode.mHeight), scanData,
                                nodePose, resultSummary);

        /* Ignore the node if the score falls below the threshold */
        if (resultSummary.mNormalizedScore <= scoreMax ||
            resultSummary.mMatchRate < this->mMatchRateThreshold) {
            /* Pop the current node from the stack */
            nodeStack.pop();
            continue;
        }

        /* If the current node is a leaf node, update the solution */
        if (currentNode.IsLeafNode()) {
            bestSensorPose = nodePose;
            scoreMax = resultSummary.mNormalizedScore;
            solutionFound = true;
        } else {
            /* Otherwise, split the current node into four new nodes */
            const int x = currentNode.mX;
            const int y = currentNode.mY;
            const int theta = currentNode.mTheta;
            const int height = currentNode.mHeight - 1;
            const int winSize = static_cast<int>(std::pow(2, height));

            /* Pop the current node from the stack */
            nodeStack.pop();

            /* Push the new nodes to the stack */
            nodeStack.emplace(x, y, theta, height);
            nodeStack.emplace(x + winSize, y, theta, height);
            nodeStack.emplace(x, y + winSize, theta, height);
            nodeStack.emplace(x + winSize, y + winSize, theta, height);
        }
    }

    /* Loop closure fails if the solution is not found */
    if (!solutionFound)
        return false;

    /* Estimate the covariance matrix */
    const Eigen::Matrix3d covMat =
        this->mCostFunc->ComputeCovariance(localMap, scanData, bestSensorPose);

    /* Compute the robot pose from the sensor pose */
    const RobotPose2D<double> bestPose =
        MoveBackward(bestSensorPose, scanData->RelativeSensorPose());

    /* Return the result */
    correspondingPose = bestPose;
    estimatedCovMat = covMat;

    return true;
}

} /* Mapping */
} /* namespace MyLidarGraphSlam */
