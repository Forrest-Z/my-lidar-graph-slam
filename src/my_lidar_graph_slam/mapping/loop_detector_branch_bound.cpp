
/* loop_detector_branch_bound.cpp */

#include <cassert>
#include <deque>
#include <limits>

#include "my_lidar_graph_slam/io/map_saver.hpp"
#include "my_lidar_graph_slam/mapping/loop_detector_branch_bound.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/* Find a loop and return a loop constraint */
bool LoopDetectorBranchBound::FindLoop(
    LoopDetectionQueryVector& loopDetectionQueries,
    LoopDetectionResultVector& loopDetectionResults)
{
    /* Clear the loop detection results */
    loopDetectionResults.clear();

    /* Do not perform loop detection if no query exists */
    if (loopDetectionQueries.empty())
        return false;

    /* Perform loop detection for each query */
    for (auto& loopDetectionQuery : loopDetectionQueries) {
        /* Retrieve the information about each query */
        const auto& poseGraphNodes = loopDetectionQuery.mPoseGraphNodes;
        auto& localMapInfo = loopDetectionQuery.mLocalMapInfo;
        const auto& localMapNode = loopDetectionQuery.mLocalMapNode;

        /* Make sure that the node is inside the local grid map */
        assert(localMapNode.Index() >= localMapInfo.mPoseGraphNodeIdxMin &&
               localMapNode.Index() <= localMapInfo.mPoseGraphNodeIdxMax);

        /* Make sure that the grid map is in finished state */
        assert(localMapInfo.mFinished);

        /* Precompute the coarser local grid maps for efficiency */
        if (!localMapInfo.mPrecomputed)
            PrecomputeGridMaps(localMapInfo, this->mNodeHeightMax);

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
bool LoopDetectorBranchBound::FindCorrespondingPose(
    const GridMapType& localMap,
    const std::map<int, PrecomputedMapType>& precompMaps,
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
    const int winSizeMax = 1 << this->mNodeHeightMax;

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
            /* Pop the current node from the stack */
            nodeStack.pop();

            /* Update the solution */
            bestSensorPose = nodePose;
            scoreMax = resultSummary.mNormalizedScore;
            solutionFound = true;
        } else {
            /* Otherwise, split the current node into four new nodes */
            const int x = currentNode.mX;
            const int y = currentNode.mY;
            const int theta = currentNode.mTheta;
            const int height = currentNode.mHeight - 1;
            const int winSize = 1 << height;

            /* Pop the current node from the stack */
            nodeStack.pop();

            /* Push the new nodes to the stack */
            nodeStack.emplace(x, y, theta, height);
            nodeStack.emplace(x + winSize, y, theta, height);
            nodeStack.emplace(x, y + winSize, theta, height);
            nodeStack.emplace(x + winSize, y + winSize, theta, height);
        }
    }

    /* Loop detection fails if the solution is not found */
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
