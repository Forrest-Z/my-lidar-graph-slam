
/* scan_matcher_real_time_correlative.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_SCAN_MATCHER_REAL_TIME_CORRELATIVE_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_SCAN_MATCHER_REAL_TIME_CORRELATIVE_HPP

#include "my_lidar_graph_slam/mapping/scan_matcher.hpp"

#include "my_lidar_graph_slam/mapping/grid_map_builder.hpp"
#include "my_lidar_graph_slam/mapping/cost_function.hpp"
#include "my_lidar_graph_slam/mapping/score_function.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

class ScanMatcherRealTimeCorrelative final : public ScanMatcher
{
public:
    /* Constructor */
    ScanMatcherRealTimeCorrelative(const CostFuncPtr& costFunc,
                                   int lowResolution,
                                   double rangeX,
                                   double rangeY,
                                   double rangeTheta,
                                   double scanRangeMax);

    /* Destructor */
    ~ScanMatcherRealTimeCorrelative() = default;

    /* Optimize the robot pose by scan matching */
    ScanMatchingSummary OptimizePose(
        const ScanMatchingQuery& queryInfo) override;

private:
    /* Compute the search step */
    void ComputeSearchStep(const GridMapBase<double>& gridMap,
                           const Sensor::ScanDataPtr<double>& scanData,
                           double& stepX,
                           double& stepY,
                           double& stepTheta);

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
    CostFuncPtr  mCostFunc;
    /* Resolution for low resolution map (in the number of grid cells) */
    int          mLowResolution;
    /* Linear (horizontal) size of the search window */
    double       mRangeX;
    /* Linear (vertical) size of the search window */
    double       mRangeY;
    /* Angular range of the search window */
    double       mRangeTheta;
    /* Maximum laser scan range considered for scan matching */
    double       mScanRangeMax;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_SCAN_MATCHER_REAL_TIME_CORRELATIVE_HPP */
