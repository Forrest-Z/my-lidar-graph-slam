
/* score_function.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_SCORE_FUNCTION_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_SCORE_FUNCTION_HPP

#include <cmath>
#include <cstdlib>
#include <memory>

#include "my_lidar_graph_slam/point.hpp"
#include "my_lidar_graph_slam/pose.hpp"
#include "my_lidar_graph_slam/grid_map/grid_map_base.hpp"
#include "my_lidar_graph_slam/sensor/sensor_data.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/* Declare types for convenience */
class ScoreFunction;
using ScoreFuncPtr = std::shared_ptr<ScoreFunction>;

class ScoreFunction
{
public:
    /*
     * Summary struct holds the details of the result
     */
    struct Summary
    {
        /* Normalized matching score */
        double mNormalizedScore;
        /* Matching score */
        double mScore;
        /* Rate of the grid cells with valid occupancy probability value */
        double mMatchRate;
    };

public:
    /* Constructor */
    ScoreFunction() = default;
    /* Destructor */
    virtual ~ScoreFunction() = default;

    /* Copy constructor (disabled) */
    ScoreFunction(const ScoreFunction&) = delete;
    /* Copy assignment operator (disabled) */
    ScoreFunction& operator=(const ScoreFunction&) = delete;
    /* Move constructor (disabled) */
    ScoreFunction(ScoreFunction&&) = delete;
    /* Move assignment operator (disabled) */
    ScoreFunction& operator=(ScoreFunction&&) = delete;

    /* Evaluate score function (matching score between scan data and map) */
    virtual void Score(
        const GridMapBase<double>& gridMap,
        const Sensor::ScanDataPtr<double>& scanData,
        const RobotPose2D<double>& mapLocalSensorPose,
        Summary& resultSummary) = 0;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_SCORE_FUNCTION_HPP */
