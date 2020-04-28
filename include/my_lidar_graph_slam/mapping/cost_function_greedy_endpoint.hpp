
/* cost_function_greedy_endpoint.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_COST_FUNCTION_GREEDY_ENDPOINT_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_COST_FUNCTION_GREEDY_ENDPOINT_HPP

#include "my_lidar_graph_slam/mapping/cost_function.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

class CostGreedyEndpoint final : public CostFunction
{
public:
    /* Constructor */
    CostGreedyEndpoint(double usableRangeMin,
                       double usableRangeMax,
                       double hitAndMissedDist,
                       double occupancyThreshold,
                       int kernelSize,
                       double scalingFactor);
    
    /* Destructor */
    ~CostGreedyEndpoint() = default;

    /* Calculate cost function (mismatch between scan data and map) */
    double Cost(const GridMapBase<double>& gridMap,
                const Sensor::ScanDataPtr<double>& scanData,
                const RobotPose2D<double>& sensorPose) override;

    /* Calculate a gradient vector */
    Eigen::Vector3d ComputeGradient(
        const GridMapBase<double>& gridMap,
        const Sensor::ScanDataPtr<double>& scanData,
        const RobotPose2D<double>& sensorPose) override;
    
    /* Calculate a covariance matrix */
    Eigen::Matrix3d ComputeCovariance(
        const GridMapBase<double>& gridMap,
        const Sensor::ScanDataPtr<double>& scanData,
        const RobotPose2D<double>& sensorPose) override;

private:
    /* Lookup table for squared distance between grid cells */

    /* Minimum laser scan range considered for calculation */
    double mUsableRangeMin;
    /* Maximum laser scan range considered for calculation */
    double mUsableRangeMax;
    /* Distance between hit and missed grid cells */
    double mHitAndMissedDist;
    /* Probability threshold for being obstructed */
    double mOccupancyThreshold;
    /* Size of searching window (in the number of grid cells) */
    int    mKernelSize;
    /* Scaling factor for cost value */
    double mScalingFactor;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_COST_FUNCTION_GREEDY_ENDPOINT_HPP */
