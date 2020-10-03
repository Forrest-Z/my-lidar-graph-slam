
/* cost_function_greedy_endpoint.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_COST_FUNCTION_GREEDY_ENDPOINT_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_COST_FUNCTION_GREEDY_ENDPOINT_HPP

#include "my_lidar_graph_slam/mapping/cost_function.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/* Type definitions for convenience */
class CostGreedyEndpoint;
using CostGreedyEndpointPtr = std::shared_ptr<CostGreedyEndpoint>;

class CostGreedyEndpoint final : public CostFunction
{
public:
    /* Constructor */
    CostGreedyEndpoint(const double usableRangeMin,
                       const double usableRangeMax,
                       const double hitAndMissedDist,
                       const double occupancyThreshold,
                       const int kernelSize,
                       const double scalingFactor,
                       const double standardDeviation);

    /* Destructor */
    ~CostGreedyEndpoint() = default;

    /* Calculate cost function (mismatch between scan data and map) */
    double Cost(
        const GridMapBase<double>& gridMap,
        const Sensor::ScanDataPtr<double>& scanData,
        const RobotPose2D<double>& mapLocalSensorPose) override;

    /* Calculate a gradient vector in a map-local coordinate frame */
    Eigen::Vector3d ComputeGradient(
        const GridMapBase<double>& gridMap,
        const Sensor::ScanDataPtr<double>& scanData,
        const RobotPose2D<double>& mapLocalSensorPose) override;

    /* Calculate a covariance matrix in a map-local coordinate frame */
    Eigen::Matrix3d ComputeCovariance(
        const GridMapBase<double>& gridMap,
        const Sensor::ScanDataPtr<double>& scanData,
        const RobotPose2D<double>& mapLocalSensorPose) override;

private:
    /* TODO: Lookup table for squared distance between grid cells */
    /* Minimum laser scan range considered for calculation */
    const double mUsableRangeMin;
    /* Maximum laser scan range considered for calculation */
    const double mUsableRangeMax;
    /* Distance between hit and missed grid cells */
    const double mHitAndMissedDist;
    /* Probability threshold for being obstructed */
    const double mOccupancyThreshold;
    /* Size of searching window (in the number of grid cells) */
    const int    mKernelSize;
    /* Standard deviation of the Gaussian distribution of the error */
    const double mStandardDeviation;
    /* Variance of the Gaussian distribution of the error */
    const double mVariance;
    /* Scaling factor for cost value */
    const double mScalingFactor;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_COST_FUNCTION_GREEDY_ENDPOINT_HPP */
