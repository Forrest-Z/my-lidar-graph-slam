
/* scan_matcher_real_time_correlative.cpp */

#include "my_lidar_graph_slam/mapping/scan_matcher_real_time_correlative.hpp"

#include <algorithm>
#include <limits>
#include <numeric>

namespace MyLidarGraphSlam {
namespace Mapping {

/* Constructor */
ScanMatcherRealTimeCorrelative::ScanMatcherRealTimeCorrelative(
    const CostFuncPtr& costFunc,
    int lowResolution,
    double rangeX,
    double rangeY,
    double rangeTheta,
    double scanRangeMax) :
    mCostFunc(costFunc),
    mLowResolution(lowResolution),
    mRangeX(rangeX),
    mRangeY(rangeY),
    mRangeTheta(rangeTheta),
    mScanRangeMax(scanRangeMax)
{
}

/* Optimize the robot pose by scan matching */
ScanMatchingSummary ScanMatcherRealTimeCorrelative::OptimizePose(
    const ScanMatchingQuery& queryInfo)
{
    /* Retrieve the query information */
    const auto& gridMap = queryInfo.mGridMap;
    const auto& scanData = queryInfo.mScanData;
    const RobotPose2D<double>& initialPose = queryInfo.mInitialPose;

    /* Precompute the grid map */
    const PrecomputedMapType precompMap = this->ComputeCoarserMap(gridMap);

    /* Optimize the robot pose by scan matching
     * Pass the minimum possible value as a score threshold to
     * search the entire window */
    return this->OptimizePose(gridMap, precompMap, scanData, initialPose,
                              std::numeric_limits<double>::min());
}

/* Optimize the robot pose by scan matching */
ScanMatchingSummary ScanMatcherRealTimeCorrelative::OptimizePose(
    const GridMapType& gridMap,
    const PrecomputedMapType& precompMap,
    const Sensor::ScanDataPtr<double>& scanData,
    const RobotPose2D<double>& initialPose,
    const double normalizedScoreThreshold) const
{
    /* Find the best pose from the search window */
    const RobotPose2D<double> sensorPose =
        Compound(initialPose, scanData->RelativeSensorPose());

    /* Determine the search step */
    double stepX;
    double stepY;
    double stepTheta;
    this->ComputeSearchStep(gridMap, scanData, stepX, stepY, stepTheta);

    /* Determine the search window */
    /* 'winX' and 'winY' are in the number of grid cells */
    const int winX = static_cast<int>(
        std::ceil(0.5 * this->mRangeX / stepX));
    const int winY = static_cast<int>(
        std::ceil(0.5 * this->mRangeY / stepY));
    const int winTheta = static_cast<int>(
        std::ceil(0.5 * this->mRangeTheta / stepTheta));

    /* Perform scan matching against the low resolution grid map */
    const double scoreThreshold =
        normalizedScoreThreshold * scanData->NumOfScans();
    double scoreMax = scoreThreshold;
    int bestWinX = -winX;
    int bestWinY = -winY;
    int bestWinTheta = -winTheta;

    /* Compute the grid cell indices for scan points */
    std::vector<Point2D<int>> scanIndices;
    scanIndices.reserve(scanData->NumOfScans());

    for (int t = -winTheta; t <= winTheta; ++t) {
        /* Compute the grid cell indices for scan points */
        const RobotPose2D<double> currentSensorPose {
            sensorPose.mX, sensorPose.mY, sensorPose.mTheta + stepTheta * t };
        this->ComputeScanIndices(
            precompMap, currentSensorPose, scanData, scanIndices);

        /* 'winX' and 'winY' are represented in the number of grid cells */
        /* For given 't', the projected scan points 'scanIndices' are
         * related by pure translation for the 'x' and 'y' search directions */
        for (int x = -winX; x <= winX; x += this->mLowResolution) {
            for (int y = -winY; y <= winY; y += this->mLowResolution) {
                /* Evaluate the matching score */
                const double score = this->ComputeScore(
                    precompMap, scanIndices, x, y);

                /* Ignore the score of the low-resolution grid cell
                 * if the score is below a maximum score */
                if (score <= scoreMax)
                    continue;

                /* Evaluate the score using the high-resolution grid map */
                /* Update the maximum score and search window index */
                this->EvaluateHighResolutionMap(
                    gridMap, scanIndices, x, y, t,
                    bestWinX, bestWinY, bestWinTheta, scoreMax);
            }
        }
    }

    /* The appropriate solution is found if the maximum score is
     * larger than (not larger than or equal to) the score threshold */
    const bool poseFound = scoreMax > scoreThreshold;
    /* Compute the best sensor pose */
    const RobotPose2D<double> bestSensorPose {
        sensorPose.mX + bestWinX * stepX,
        sensorPose.mY + bestWinY * stepY,
        sensorPose.mTheta + bestWinTheta * stepTheta };
    /* Evaluate the cost value */
    const double costVal = this->mCostFunc->Cost(
        gridMap, scanData, bestSensorPose);

    /* Compute the normalized cost value */
    const double normalizedCost = costVal / scanData->NumOfScans();
    /* Compute the estimated robot pose in a world frame */
    const RobotPose2D<double> estimatedPose =
        MoveBackward(bestSensorPose, scanData->RelativeSensorPose());
    /* Compute the pose covariance matrix */
    const Eigen::Matrix3d estimatedCovariance =
        this->mCostFunc->ComputeCovariance(gridMap, scanData, bestSensorPose);

    /* Return the normalized cost value, the estimated robot pose,
     * and the estimated pose covariance matrix in a world frame */
    return ScanMatchingSummary {
        poseFound, normalizedCost, initialPose,
        estimatedPose, estimatedCovariance };
}

/* Precompute a coarser grid map for scan matching */
PrecomputedMapType ScanMatcherRealTimeCorrelative::ComputeCoarserMap(
    const GridMapType& gridMap) const
{
    /* Create a coarser grid map with the specified resolution */
    return PrecomputeGridMap(gridMap, this->mLowResolution);
}

/* Compute the search step */
void ScanMatcherRealTimeCorrelative::ComputeSearchStep(
    const GridMapBase<double>& gridMap,
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

/* Compute the grid cell indices for scan points */
void ScanMatcherRealTimeCorrelative::ComputeScanIndices(
    const PrecomputedMapType& precompMap,
    const RobotPose2D<double>& sensorPose,
    const Sensor::ScanDataPtr<double>& scanData,
    std::vector<Point2D<int>>& scanIndices) const
{
    /* Compute the grid cell indices for scan points */
    scanIndices.clear();

    const std::size_t numOfScans = scanData->NumOfScans();

    for (std::size_t i = 0; i < numOfScans; ++i) {
        const double range = scanData->RangeAt(i);

        if (range >= this->mScanRangeMax)
            continue;

        Point2D<double> hitPoint =
            scanData->HitPoint(sensorPose, i);
        Point2D<int> hitIdx =
            precompMap.WorldCoordinateToGridCellIndex(hitPoint);
        scanIndices.push_back(std::move(hitIdx));
    }

    return;
}

/* Compute the scan matching score based on the already projected
 * scan points (indices) and index offsets */
double ScanMatcherRealTimeCorrelative::ComputeScore(
    const GridMapBase<double>& gridMap,
    const std::vector<Point2D<int>>& scanIndices,
    const int offsetX,
    const int offsetY) const
{
    /* Evaluate the matching score based on the occupancy probability value */
    const double unknownVal = gridMap.UnknownValue();
    double sumScore = 0.0;

    for (const auto& hitIdx : scanIndices) {
        const double mapVal = gridMap.Value(
            hitIdx.mX + offsetX, hitIdx.mY + offsetY, unknownVal);
        sumScore += mapVal;
    }

    return sumScore;
}

/* Evaluate the matching score using high-resolution grid map */
void ScanMatcherRealTimeCorrelative::EvaluateHighResolutionMap(
    const GridMapBase<double>& gridMap,
    const std::vector<Point2D<int>>& scanIndices,
    const int offsetX,
    const int offsetY,
    const int offsetTheta,
    int& maxWinX,
    int& maxWinY,
    int& maxWinTheta,
    double& maxScore) const
{
    /* Search inside the relatively small area */
    for (int x = offsetX; x < offsetX + this->mLowResolution; ++x) {
        for (int y = offsetY; y < offsetY + this->mLowResolution; ++y) {
            /* Evaluate matching score */
            const double score =
                this->ComputeScore(gridMap, scanIndices, x, y);

            /* Update the maximum score and search window index */
            if (maxScore < score) {
                maxScore = score;
                maxWinX = x;
                maxWinY = y;
                maxWinTheta = offsetTheta;
            }
        }
    }

    return;
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
