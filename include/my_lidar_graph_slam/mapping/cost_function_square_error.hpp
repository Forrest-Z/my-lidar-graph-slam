
/* cost_function_square_error.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_COST_FUNCTION_SQUARE_ERROR_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_COST_FUNCTION_SQUARE_ERROR_HPP

#include "my_lidar_graph_slam/mapping/cost_function.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/* Type definitions for convenience */
class CostSquareError;
using CostSquareErrorPtr = std::shared_ptr<CostSquareError>;

class CostSquareError final : public CostFunction
{
public:
    /* Constructor */
    CostSquareError(const double usableRangeMin,
                    const double usableRangeMax);

    /* Destructor */
    ~CostSquareError() = default;

    /* Calculate cost function (mismatch between scan data and map) */
    double Cost(
        const GridMapBase<double>& gridMap,
        const Sensor::ScanDataPtr<double>& scanData,
        const RobotPose2D<double>& mapLocalSensorPose) override;

    /* Calculate a gradient of the cost function
     * with respect to the robot pose */
    Eigen::Vector3d ComputeGradient(
        const GridMapBase<double>& gridMap,
        const Sensor::ScanDataPtr<double>& scanData,
        const RobotPose2D<double>& mapLocalSensorPose) override;

    /* Calculate a covariance matrix */
    Eigen::Matrix3d ComputeCovariance(
        const GridMapBase<double>& gridMap,
        const Sensor::ScanDataPtr<double>& scanData,
        const RobotPose2D<double>& mapLocalSensorPose) override;

    /* Calculate a gradient of the cost function
     * with respect to the robot pose numerically */
    Eigen::Vector3d ComputeNumericalGradient(
        const GridMapBase<double>& gridMap,
        const Sensor::ScanDataPtr<double>& scanData,
        const RobotPose2D<double>& mapLocalSensorPose);

    /* Calculate a gradient of the smoothed map function
     * with respect to the specified position on the map */
    Eigen::Vector2d ComputeMapGradient(
        const GridMapBase<double>& gridMap,
        const Point2D<double>& localMapPos);

    /* Calculate a gradient of the smoothed map function
     * at the specified scan point with respect to the robot pose */
    Eigen::Vector3d ComputeMapGradient(
        const GridMapBase<double>& gridMap,
        const RobotPose2D<double>& mapLocalSensorPose,
        const double scanRange,
        const double scanAngle);

    /* Calculate a gradient of the smoothed map function
     * at the specified scan point with respect to the robot pose */
    Eigen::Vector3d ComputeMapNumericalGradient(
        const GridMapBase<double>& gridMap,
        const RobotPose2D<double>& mapLocalSensorPose,
        const double scanRange,
        const double scanAngle);

    /* Calculate the smoothed occupancy probability value
     * using bicubic interpolation */
    double ComputeSmoothedValue(
        const GridMapBase<double>& gridMap,
        const Point2D<double>& gridCellIdx) const;

private:
    /* Minimum laser scan range considered for calculation */
    const double mUsableRangeMin;
    /* Maximum laser scan range considered for calculation */
    const double mUsableRangeMax;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_COST_FUNCTION_SQUARE_ERROR_HPP */
