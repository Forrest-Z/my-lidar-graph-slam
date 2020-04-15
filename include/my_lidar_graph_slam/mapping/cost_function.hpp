
/* cost_function.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_COST_FUNCTION_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_COST_FUNCTION_HPP

#include <cmath>
#include <cstdlib>

#include "my_lidar_graph_slam/point.hpp"
#include "my_lidar_graph_slam/pose.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

template <typename T, typename U>
class CostFunction
{
public:
    /* Type definitions */
    using GridMapType = T;
    using ScanType = U;

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
    virtual double Cost(const GridMapType& gridMap,
                        const ScanType& scanData,
                        const RobotPose2D<double>& sensorPose) = 0;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_COST_FUNCTION_HPP */
