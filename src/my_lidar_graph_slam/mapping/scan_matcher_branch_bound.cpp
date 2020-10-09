
/* scan_matcher_branch_bound.cpp */

#include "my_lidar_graph_slam/mapping/scan_matcher_branch_bound.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/* Constructor */
ScanMatcherBranchBound::ScanMatcherBranchBound(
    const std::shared_ptr<ScorePixelAccurate>& scoreFunc,
    const CostFuncPtr& costFunc,
    const int nodeHeightMax,
    const double rangeX,
    const double rangeY,
    const double rangeTheta,
    const double scanRangeMax) :
    mScoreFunc(scoreFunc),
    mCostFunc(costFunc),
    mNodeHeightMax(nodeHeightMax),
    mRangeX(rangeX),
    mRangeY(rangeY),
    mRangeTheta(rangeTheta),
    mScanRangeMax(scanRangeMax)
{
}

/* Optimize the robot pose by scan matching */
ScanMatchingSummary ScanMatcherBranchBound::OptimizePose(
    const ScanMatchingQuery& queryInfo)
{
    /* Retrieve the query information */
    const auto& gridMap = queryInfo.mGridMap;
    const auto& scanData = queryInfo.mScanData;
    const RobotPose2D<double>& mapLocalInitialPose =
        queryInfo.mMapLocalInitialPose;

    /* Precompute the coarser grid maps */
    const std::vector<ConstMapType> precompMaps =
        this->ComputeCoarserMaps(gridMap);

    /* Optimize the robot pose by scan matching */
    return this->OptimizePose(gridMap, precompMaps, scanData,
                              mapLocalInitialPose, 0.0);
}

/* Optimize the robot pose by scan matching */
ScanMatchingSummary ScanMatcherBranchBound::OptimizePose(
    const GridMapType& gridMap,
    const std::vector<ConstMapType>& precompMaps,
    const Sensor::ScanDataPtr<double>& scanData,
    const RobotPose2D<double>& mapLocalInitialPose,
    const double normalizedScoreThreshold) const
{
    /* Find the best pose from the search window
     * that maximizes the matching score value */
    const RobotPose2D<double> mapLocalSensorPose =
        Compound(mapLocalInitialPose, scanData->RelativeSensorPose());

    /* Determine the search window step */
    double stepX;
    double stepY;
    double stepTheta;
    this->ComputeSearchStep(gridMap, scanData, stepX, stepY, stepTheta);

    /* Determine the search window */
    const int winX = static_cast<int>(
        std::ceil(0.5 * this->mRangeX / stepX));
    const int winY = static_cast<int>(
        std::ceil(0.5 * this->mRangeY / stepY));
    const int winTheta = static_cast<int>(
        std::ceil(0.5 * this->mRangeTheta / stepTheta));

    /* Setup the best score */
    const double scoreThreshold =
        normalizedScoreThreshold * scanData->NumOfScans();
    double scoreMax = scoreThreshold;

    /* Setup the best pose */
    RobotPose2D<double> bestSensorPose = mapLocalSensorPose;

    /* Initialize a stack with nodes covering the entire search window */
    std::stack<Node> nodeStack;
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
            mapLocalSensorPose.mX + currentNode.mX * stepX,
            mapLocalSensorPose.mY + currentNode.mY * stepY,
            mapLocalSensorPose.mTheta + currentNode.mTheta * stepTheta };

        /* Calculate node score */
        const ScoreFunction::Summary resultSummary = this->mScoreFunc->Score(
            precompMaps.at(currentNode.mHeight), scanData, nodePose);

        /* Ignore the node if the score falls below the threshold */
        if (resultSummary.mScore <= scoreMax) {
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
            scoreMax = resultSummary.mScore;
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

    /* The appropriate pose is found if the maximum score is larger than
     * (not larger than or equal to) the score threshold */
    const bool poseFound = scoreMax > scoreThreshold;

    /* Evaluate the cost value */
    const double costVal = this->mCostFunc->Cost(
        gridMap, scanData, bestSensorPose);
    /* Compute the normalized cost value */
    const double normalizedCost = costVal / scanData->NumOfScans();

    /* Compute the estimated robot pose in a map-local coordinate frame */
    const RobotPose2D<double> estimatedPose =
        MoveBackward(bestSensorPose, scanData->RelativeSensorPose());
    /* Compute the pose covariance matrix in a map-local coordinate frame */
    const Eigen::Matrix3d estimatedCovariance =
        this->mCostFunc->ComputeCovariance(gridMap, scanData, bestSensorPose);

    /* Return the normalized cost value, the estimated robot pose,
     * and the estimated pose covariance matrix in a map-local frame */
    return ScanMatchingSummary {
        poseFound, normalizedCost, mapLocalInitialPose,
        estimatedPose, estimatedCovariance };
}

/* Precompute multiple coarser grid maps for scan matching */
std::vector<ConstMapType> ScanMatcherBranchBound::ComputeCoarserMaps(
    const GridMapType& gridMap) const
{
    /* Map with the tree height (key) and the coarser grid map (value) */
    std::vector<ConstMapType> precompMaps;
    /* Create multiple coarser grid maps */
    PrecomputeGridMaps(gridMap, precompMaps, this->mNodeHeightMax);
    /* Return the grid map pyramid */
    return precompMaps;
}

/* Compute the search window step */
void ScanMatcherBranchBound::ComputeSearchStep(
    const GridMapType& gridMap,
    const Sensor::ScanDataPtr<double>& scanData,
    double& stepX,
    double& stepY,
    double& stepTheta) const
{
    /* Determine the search step */
    const double mapResolution = gridMap.Resolution();
    const auto maxRangeIt = std::max_element(
        scanData->Ranges().cbegin(), scanData->Ranges().cend());
    const double maxRange = std::min(*maxRangeIt, this->mScanRangeMax);
    const double theta = mapResolution / maxRange;

    stepX = mapResolution;
    stepY = mapResolution;
    stepTheta = std::acos(1.0 - 0.5 * theta * theta);

    return;
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
