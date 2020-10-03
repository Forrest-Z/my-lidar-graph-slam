
/* cost_function.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_COST_FUNCTION_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_COST_FUNCTION_HPP

#include <cmath>
#include <cstdlib>
#include <memory>

#include <Eigen/Core>

#include "my_lidar_graph_slam/point.hpp"
#include "my_lidar_graph_slam/pose.hpp"
#include "my_lidar_graph_slam/grid_map/grid_map_base.hpp"
#include "my_lidar_graph_slam/sensor/sensor_data.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/* Type definitions for convenience */
class CostFunction;
using CostFuncPtr = std::shared_ptr<CostFunction>;

class CostFunction
{
public:
    /* Constructor */
    CostFunction() = default;
    /* Destructor */
    virtual ~CostFunction() = default;

    /* Copy constructor (disabled) */
    CostFunction(const CostFunction&) = delete;
    /* Copy assignment operator (disabled) */
    CostFunction& operator=(const CostFunction&) = delete;
    /* Move constructor (disabled) */
    CostFunction(CostFunction&&) = delete;
    /* Move assignment operator (disabled) */
    CostFunction& operator=(CostFunction&&) = delete;

    /* Calculate cost function (mismatch between scan data and map) */
    virtual double Cost(
        const GridMapBase<double>& gridMap,
        const Sensor::ScanDataPtr<double>& scanData,
        const RobotPose2D<double>& mapLocalSensorPose) = 0;

    /* Calculate a gradient vector in a map-local coordinate frame */
    virtual Eigen::Vector3d ComputeGradient(
        const GridMapBase<double>& gridMap,
        const Sensor::ScanDataPtr<double>& scanData,
        const RobotPose2D<double>& mapLocalSensorPose) = 0;

    /* Calculate a covariance matrix in a map-local coordinate frame */
    virtual Eigen::Matrix3d ComputeCovariance(
        const GridMapBase<double>& gridMap,
        const Sensor::ScanDataPtr<double>& scanData,
        const RobotPose2D<double>& mapLocalSensorPose) = 0;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_COST_FUNCTION_HPP */
