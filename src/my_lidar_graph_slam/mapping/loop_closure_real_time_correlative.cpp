
/* loop_closure_real_time_correlative.cpp */

#include "my_lidar_graph_slam/mapping/loop_closure_real_time_correlative.hpp"

#include <algorithm>
#include <cassert>
#include <limits>
#include <numeric>

#include "my_lidar_graph_slam/metric/metric.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/* Find a loop and return a loop constraint */
bool LoopClosureRealTimeCorrelative::FindLoop(
    std::shared_ptr<GridMapBuilder>& gridMapBuilder,
    const std::shared_ptr<PoseGraph>& poseGraph,
    RobotPose2D<double>& relPose,
    int& startNodeIdx,
    int& endNodeIdx,
    Eigen::Matrix3d& estimatedCovMat)
{
    /* Retrieve the current robot pose and scan data */
    const auto& currentNode = poseGraph->LatestNode();
    const RobotPose2D<double>& currentPose = currentNode.Pose();
    const Sensor::ScanDataPtr<double>& currentScanData = currentNode.ScanData();
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

    auto& candidateMapInfo = gridMapBuilder->LocalMapAt(candidateMapIdx);
    const auto& candidateMap = candidateMapInfo.mMap;
    const auto& candidateNode = poseGraph->NodeAt(candidateNodeIdx);
    const RobotPose2D<double>& candidateNodePose = candidateNode.Pose();

    /* The local grid map used for loop closure must be finished */
    assert(candidateMapInfo.mFinished);

    /* Precompute a low-resolution grid map */
    if (!candidateMapInfo.mPrecomputed) {
        /* Precompute a grid map */
        GridMapBuilder::PrecomputedMapType precompMap =
            PrecomputeGridMap(candidateMap, this->mLowResolution);
        /* Append the newly created grid map */
        candidateMapInfo.mPrecomputedMaps.emplace(0, std::move(precompMap));
        /* Mark the local map as precomputed */
        candidateMapInfo.mPrecomputed = true;
    }

    /* The number of the precomputed grid maps must be 1 */
    assert(candidateMapInfo.mPrecomputedMaps.size() == 1);

    RobotPose2D<double> correspondingPose;
    Eigen::Matrix3d covMat;
    const bool loopFound = this->FindCorrespondingPose(
        candidateMapInfo, currentScanData, currentPose,
        correspondingPose, covMat);

    /* Do not create loop constraint if loop closure failed */
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

/* Compute the search step */
void LoopClosureRealTimeCorrelative::ComputeSearchStep(
    const GridMapBuilder::GridMapType& gridMap,
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
void LoopClosureRealTimeCorrelative::ComputeScanIndices(
    const GridMapBuilder::PrecomputedMapType& gridMap,
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
            gridMap.WorldCoordinateToGridCellIndex(hitPoint);
        scanIndices.push_back(std::move(hitIdx));
    }

    return;
}

/* Compute the scan matching score based on the already projected
 * scan points (indices) and index offsets */
double LoopClosureRealTimeCorrelative::ComputeScore(
    const GridMapBase<double>& gridMap,
    const std::vector<Point2D<int>>& scanIndices,
    const int offsetX,
    const int offsetY) const
{
    /* Compute the matching score based on the occupancy probability value */
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
void LoopClosureRealTimeCorrelative::EvaluateHighResolutionMap(
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
    /* Search inside the square area of 'mLowResolution' * 'mLowResolution' */
    for (int x = offsetX; x < offsetX + this->mLowResolution; ++x) {
        for (int y = offsetY; y < offsetY + this->mLowResolution; ++y) {
            /* Evaluate the matching score */
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

/* Find a corresponding pose of the current robot pose
 * from the loop-closure candidate local grid map */
bool LoopClosureRealTimeCorrelative::FindCorrespondingPose(
    const GridMapBuilder::LocalMapInfo& localMapInfo,
    const Sensor::ScanDataPtr<double>& scanData,
    const RobotPose2D<double>& robotPose,
    RobotPose2D<double>& correspondingPose,
    Eigen::Matrix3d& estimatedCovMat) const
{
    /* Retrieve the local grid map */
    const GridMapBuilder::GridMapType& localMap = localMapInfo.mMap;
    /* Retrieve the precomputed low-resolution grid map */
    assert(localMapInfo.mPrecomputedMaps.size() == 1);
    const GridMapBuilder::PrecomputedMapType& precompMap =
        localMapInfo.mPrecomputedMaps.at(0);

    /* Find the best pose from the search window */
    const RobotPose2D<double> sensorPose =
        Compound(robotPose, scanData->RelativeSensorPose());

    /* Determine the search window step */
    double stepX;
    double stepY;
    double stepTheta;
    this->ComputeSearchStep(localMap, scanData, stepX, stepY, stepTheta);

    /* Determine the search window */
    const int winX = static_cast<int>(
        std::ceil(0.5 * this->mRangeX / stepX));
    const int winY = static_cast<int>(
        std::ceil(0.5 * this->mRangeY / stepY));
    const int winTheta = static_cast<int>(
        std::ceil(0.5 * this->mRangeTheta / stepTheta));

    /* Perform loop closure against the low-resolution grid map */
    /* Initialize the maximum score with the score threshold,
     * which is not normalized */
    const double scoreThreshold =
        this->mScoreThreshold * scanData->NumOfScans();
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
                    localMap, scanIndices, x, y, t,
                    bestWinX, bestWinY, bestWinTheta, scoreMax);
            }
        }
    }

    /* Update metrics */
    Metric::MetricManager* const pMetric = Metric::MetricManager::Instance();
    auto& histMetrics = pMetric->HistogramMetrics();
    histMetrics("LoopClosureMaxScore")->Observe(scoreMax);

    /* Loop closure fails if the score does not exceed the threshold */
    if (scoreMax <= scoreThreshold)
        return false;

    /* Compute the best sensor pose */
    const RobotPose2D<double> bestSensorPose {
        sensorPose.mX + bestWinX * stepX,
        sensorPose.mY + bestWinY * stepY,
        sensorPose.mTheta + bestWinTheta * stepTheta };

    /* Compute the pose covariance matrix */
    const Eigen::Matrix3d covMat = this->mCostFunc->ComputeCovariance(
        localMap, scanData, bestSensorPose);
    /* Compute the robot pose from the sensor pose */
    const RobotPose2D<double> bestPose =
        MoveBackward(bestSensorPose, scanData->RelativeSensorPose());

    /* Return the result */
    correspondingPose = bestPose;
    estimatedCovMat = covMat;

    /* Update metrics */
    histMetrics("LoopClosureMaxScoreSuccess")->Observe(scoreMax);

    return true;
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
