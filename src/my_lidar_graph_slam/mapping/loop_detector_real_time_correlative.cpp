
/* loop_detector_real_time_correlative.cpp */

#include "my_lidar_graph_slam/mapping/loop_detector_real_time_correlative.hpp"

#include <algorithm>
#include <cassert>
#include <limits>
#include <numeric>

namespace MyLidarGraphSlam {
namespace Mapping {

/* Find a loop and return a loop constraint */
void LoopDetectorRealTimeCorrelative::Detect(
    LoopDetectionQueryVector& loopDetectionQueries,
    LoopDetectionResultVector& loopDetectionResults)
{
    /* Clear the loop detection results */
    loopDetectionResults.clear();

    /* Do not perform loop detection if no query exists */
    if (loopDetectionQueries.empty())
        return;

    /* Perform loop detection for each query */
    for (auto& loopDetectionQuery : loopDetectionQueries) {
        /* Retrieve the information for each query */
        const auto& poseGraphNodes = loopDetectionQuery.mPoseGraphNodes;
        auto& localMapInfo = loopDetectionQuery.mLocalMapInfo;
        const auto& localMapNode = loopDetectionQuery.mLocalMapNode;

        /* Make sure that the node is inside the local grid map */
        assert(localMapNode.Index() >= localMapInfo.mPoseGraphNodeIdxMin &&
               localMapNode.Index() <= localMapInfo.mPoseGraphNodeIdxMax);

        /* Make sure that the grid map is in finished state */
        assert(localMapInfo.mFinished);

        /* Precompute a low-resolution grid map */
        if (!localMapInfo.mPrecomputed) {
            /* Precompute a coarser grid map */
            PrecomputedMapType precompMap =
                PrecomputeGridMap(localMapInfo.mMap, this->mLowResolution);
            /* Append the newly created grid map */
            localMapInfo.mPrecomputedMaps.emplace(0, std::move(precompMap));
            /* Mark the current local map as precomputed */
            localMapInfo.mPrecomputed = true;
        }

        /* The local grid map should have only one precomputed grid map */
        assert(localMapInfo.mPrecomputedMaps.size() == 1);

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
            /* Relative pose of the loop closing edge */
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

    return;
}

/* Compute the search step */
void LoopDetectorRealTimeCorrelative::ComputeSearchStep(
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

/* Compute the grid cell indices for scan points */
void LoopDetectorRealTimeCorrelative::ComputeScanIndices(
    const PrecomputedMapType& gridMap,
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
double LoopDetectorRealTimeCorrelative::ComputeScore(
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
void LoopDetectorRealTimeCorrelative::EvaluateHighResolutionMap(
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
bool LoopDetectorRealTimeCorrelative::FindCorrespondingPose(
    const GridMapType& localMap,
    const std::map<int, PrecomputedMapType>& precompMaps,
    const Sensor::ScanDataPtr<double>& scanData,
    const RobotPose2D<double>& robotPose,
    RobotPose2D<double>& correspondingPose,
    Eigen::Matrix3d& estimatedCovMat) const
{
    /* Retrieve the precomputed low-resolution grid map */
    assert(precompMaps.size() == 1);
    const PrecomputedMapType& precompMap = precompMaps.at(0);

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

    return true;
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
