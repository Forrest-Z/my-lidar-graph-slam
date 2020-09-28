
/* loop_detector_real_time_correlative.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_DETECTOR_REAL_TIME_CORRELATIVE_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_DETECTOR_REAL_TIME_CORRELATIVE_HPP

#include "my_lidar_graph_slam/mapping/cost_function.hpp"
#include "my_lidar_graph_slam/mapping/score_function.hpp"
#include "my_lidar_graph_slam/mapping/loop_detector.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

class LoopDetectorRealTimeCorrelative final : public LoopDetector
{
public:
    /* Constructor */
    LoopDetectorRealTimeCorrelative(const CostFuncPtr& costFunc,
                                    int lowResolution,
                                    double rangeX,
                                    double rangeY,
                                    double rangeTheta,
                                    double scanRangeMax,
                                    double scoreThreshold) :
        mCostFunc(costFunc),
        mLowResolution(lowResolution),
        mRangeX(rangeX),
        mRangeY(rangeY),
        mRangeTheta(rangeTheta),
        mScanRangeMax(scanRangeMax),
        mScoreThreshold(scoreThreshold) { }

    /* Destructor */
    ~LoopDetectorRealTimeCorrelative() = default;

    /* Find a loop and return a loop constraint */
    void Detect(
        LoopDetectionQueryVector& loopDetectionQueries,
        LoopDetectionResultVector& loopDetectionResults) override;

private:
    /* Compute the search step */
    void ComputeSearchStep(
        const GridMapType& gridMap,
        const Sensor::ScanDataPtr<double>& scanData,
        double& stepX,
        double& stepY,
        double& stepTheta) const;

    /* Compute the grid cell indices for scan points */
    void ComputeScanIndices(
        const PrecomputedMapType& precompMap,
        const RobotPose2D<double>& sensorPose,
        const Sensor::ScanDataPtr<double>& scanData,
        std::vector<Point2D<int>>& scanIndices) const;

    /* Compute the scan matching score based on the already projected
     * scan points (indices) and index offsets */
    double ComputeScore(
        const GridMapBase<double>& gridMap,
        const std::vector<Point2D<int>>& scanIndices,
        const int offsetX,
        const int offsetY) const;

    /* Evaluate the matching score using high-resolution grid map */
    void EvaluateHighResolutionMap(
        const GridMapBase<double>& gridMap,
        const std::vector<Point2D<int>>& scanIndices,
        const int offsetX,
        const int offsetY,
        const int offsetTheta,
        int& maxWinX,
        int& maxWinY,
        int& maxWinTheta,
        double& maxScore) const;

    /* Find a corresponding pose of the current robot pose
     * from the loop-closure candidate local grid map */
    bool FindCorrespondingPose(
        const GridMapType& localMap,
        const std::map<int, PrecomputedMapType>& precompMaps,
        const Sensor::ScanDataPtr<double>& scanData,
        const RobotPose2D<double>& robotPose,
        RobotPose2D<double>& correspondingPose,
        Eigen::Matrix3d& estimatedCovMat) const;

private:
    /* Cost function */
    CostFuncPtr mCostFunc;
    /* Resolution for low resolution map (in the number of grid cells) */
    int         mLowResolution;
    /* Linear (horizontal) size of the searching window */
    double      mRangeX;
    /* Linear (vertical) size of the searching window */
    double      mRangeY;
    /* Angular size of the search window */
    double      mRangeTheta;
    /* Maximum laser scan range considered for loop closure */
    double      mScanRangeMax;
    /* Normalized matching score threshold for loop closure */
    double      mScoreThreshold;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_DETECTOR_REAL_TIME_CORRELATIVE_HPP */
