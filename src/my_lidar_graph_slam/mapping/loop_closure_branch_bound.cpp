
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
        this->PrecomputeGridMaps(gridMapBuilder, candidateMapIdx);

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

/* Precompute grid maps for efficient loop closure */
void LoopClosureBranchBound::PrecomputeGridMaps(
    GridMapBuilderPtr& gridMapBuilder,
    int mapIdx) const
{
    /* Retrieve the local map */
    auto& localMapInfo = gridMapBuilder->LocalMapAt(mapIdx);
    auto& precomputedMaps = localMapInfo.mPrecomputedMaps;
    const GridMapType& localMap = localMapInfo.mMap;

    /* The local grid map must be finished */
    assert(localMapInfo.mFinished);

    /* Create the temporary grid map to store the intermediate result */
    const double mapResolution = localMap.MapResolution();
    const Point2D<double>& minPos = localMap.MinPos();
    const Point2D<double> maxPos {
        minPos.mX + mapResolution * localMap.NumOfGridCellsX(),
        minPos.mY + mapResolution * localMap.NumOfGridCellsY() };
    PrecomputedMapType tmpMap {
        localMap.MapResolution(), localMap.PatchSize(),
        minPos.mX, minPos.mY, maxPos.mX, maxPos.mY };

    /* Make sure that the newly created grid map covers the local map */
    assert(tmpMap.MinPos() == localMap.MinPos());
    assert(tmpMap.NumOfPatchesX() >= localMap.NumOfPatchesX());
    assert(tmpMap.NumOfPatchesY() >= localMap.NumOfPatchesY());

    /* Compute a grid map for each node height */
    for (int nodeHeight = 0, winSize = 1;
         nodeHeight <= this->mNodeHeightMax; ++nodeHeight, winSize *= 2) {
        /* Each pixel stores the maximum of the occupancy probability
         * values of the 2^h * 2^h box of pixels beginning there */
        /* Create a new grid map */
        PrecomputedMapType precompMap {
            localMap.MapResolution(), localMap.PatchSize(),
            minPos.mX, minPos.mY, maxPos.mX, maxPos.mY };

        /* Check the map size */
        assert(precompMap.MinPos() == tmpMap.MinPos());
        assert(precompMap.NumOfPatchesX() >= tmpMap.NumOfPatchesX());
        assert(precompMap.NumOfPatchesY() >= tmpMap.NumOfPatchesY());

        /* Store the maximum of the 2^h pixel wide row */
        this->SlidingWindowMaxRow(localMap, tmpMap, winSize);
        /* Store the maximum of the 2^h pixel wide column */
        this->SlidingWindowMaxCol(tmpMap, precompMap, winSize);

        /* Append the newly created grid map */
        precomputedMaps.emplace(nodeHeight, std::move(precompMap));
    }

    /* Mark the local map as precomputed */
    localMapInfo.mPrecomputed = true;
}

/* Compute the maximum of a 2^h pixel wide row starting at each pixel */
void LoopClosureBranchBound::SlidingWindowMaxRow(
    const GridMapType& gridMap,
    PrecomputedMapType& tmpMap,
    int winSize) const
{
    /* Compute the maximum for each column */
    const double unknownVal = GridMapType::GridCellType::Unknown;
    int colIdx = 0;

    std::function<double(int)> inFunc =
        [&colIdx, &gridMap, unknownVal](int rowIdx) {
        return gridMap.Value(colIdx, rowIdx, unknownVal);
    };

    std::function<void(int, double)> outFunc =
        [&colIdx, &gridMap, &tmpMap, unknownVal](int rowIdx, double maxVal) {
        const Point2D<int> patchIdx =
            gridMap.GridCellIndexToPatchIndex(colIdx, rowIdx);
        const bool isAllocated =
            gridMap.PatchAt(patchIdx).IsAllocated();

        if (isAllocated || maxVal != unknownVal)
            tmpMap.Update(colIdx, rowIdx, maxVal);
    };

    /* Apply the sliding window maximum function */
    for (colIdx = 0; colIdx < gridMap.NumOfGridCellsX(); ++colIdx)
        SlidingWindowMax(inFunc, outFunc, gridMap.NumOfGridCellsY(), winSize);
}

/* Compute the maximum of a 2^h pixel wide column starting at each pixel */
void LoopClosureBranchBound::SlidingWindowMaxCol(
    const PrecomputedMapType& tmpMap,
    PrecomputedMapType& precompMap,
    int winSize) const
{
    /* Compute the maximum for each row */
    const double unknownVal = GridMapType::GridCellType::Unknown;
    int rowIdx = 0;

    std::function<double(int)> inFunc =
        [&rowIdx, &tmpMap, unknownVal](int colIdx) {
        return tmpMap.Value(colIdx, rowIdx, unknownVal);
    };

    std::function<void(int, double)> outFunc =
        [&rowIdx, &tmpMap, &precompMap, unknownVal](int colIdx, double maxVal) {
        const Point2D<int> patchIdx =
            tmpMap.GridCellIndexToPatchIndex(colIdx, rowIdx);
        const bool isAllocated =
            tmpMap.PatchAt(patchIdx).IsAllocated();

        if (isAllocated || maxVal != unknownVal)
            precompMap.Update(colIdx, rowIdx, maxVal);
    };

    /* Apply the sliding window maximum function */
    for (rowIdx = 0; rowIdx < tmpMap.NumOfGridCellsY(); ++rowIdx)
        SlidingWindowMax(inFunc, outFunc, tmpMap.NumOfGridCellsX(), winSize);
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
    const double mapResolution = localMap.MapResolution();
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
        ScoreFunctionType::Summary resultSummary;
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
