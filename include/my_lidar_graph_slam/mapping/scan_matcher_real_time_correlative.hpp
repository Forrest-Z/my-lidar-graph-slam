
/* scan_matcher_real_time_correlative.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_SCAN_MATCHER_REAL_TIME_CORRELATIVE_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_SCAN_MATCHER_REAL_TIME_CORRELATIVE_HPP

#include "my_lidar_graph_slam/mapping/scan_matcher.hpp"

#include "my_lidar_graph_slam/mapping/cost_function.hpp"
#include "my_lidar_graph_slam/mapping/grid_map_builder.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

class ScanMatcherRealTimeCorrelative final : public ScanMatcher
{
public:
    /* Constructor */
    ScanMatcherRealTimeCorrelative(
        const CostFuncPtr& costFunc,
        const int lowResolution,
        const double rangeX,
        const double rangeY,
        const double rangeTheta,
        const double scanRangeMax);

    /* Destructor */
    ~ScanMatcherRealTimeCorrelative() = default;

    /* Optimize the robot pose by scan matching */
    ScanMatchingSummary OptimizePose(
        const ScanMatchingQuery& queryInfo) override;

    /* Optimize the robot pose by scan matching */
    ScanMatchingSummary OptimizePose(
        const GridMapType& gridMap,
        const PrecomputedMapType& precompMap,
        const Sensor::ScanDataPtr<double>& scanData,
        const RobotPose2D<double>& initialPose,
        const double normalizedScoreThreshold) const;

    /* Precompute a coarser grid map for scan matching */
    PrecomputedMapType ComputeCoarserMap(
        const GridMapType& gridMap) const;

private:
    /* Compute the search step */
    void ComputeSearchStep(const GridMapBase<double>& gridMap,
                           const Sensor::ScanDataPtr<double>& scanData,
                           double& stepX,
                           double& stepY,
                           double& stepTheta) const;

    /* Compute the grid cell indices for scan points */
    void ComputeScanIndices(const PrecomputedMapType& precompMap,
                            const RobotPose2D<double>& sensorPose,
                            const Sensor::ScanDataPtr<double>& scanData,
                            std::vector<Point2D<int>>& scanIndices) const;

    /* Compute the scan matching score based on the already projected
     * scan points (indices) and index offsets */
    double ComputeScore(const GridMapBase<double>& gridMap,
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

private:
    /* Cost function just for calculating the pose covariance matrix */
    const CostFuncPtr mCostFunc;
    /* Resolution for low resolution map (in the number of grid cells) */
    const int         mLowResolution;
    /* Linear (horizontal) size of the searching window */
    const double      mRangeX;
    /* Linear (vertical) size of the searching window */
    const double      mRangeY;
    /* Angular range of the searching window */
    const double      mRangeTheta;
    /* Maximum laser scan range considered for scan matching */
    const double      mScanRangeMax;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_SCAN_MATCHER_REAL_TIME_CORRELATIVE_HPP */
