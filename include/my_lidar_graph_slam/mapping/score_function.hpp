
/* score_function.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_SCORE_FUNCTION_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_SCORE_FUNCTION_HPP

#include "my_lidar_graph_slam/pose.hpp"
#include "my_lidar_graph_slam/grid_map/binary_bayes_grid_cell.hpp"
#include "my_lidar_graph_slam/grid_map/grid_map.hpp"
#include "my_lidar_graph_slam/sensor/sensor_data.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

class ScoreFunction
{
public:
    /* Type definitions */
    using GridMapType = GridMap<BinaryBayesGridCell<double>>;
    using ScanPtr = Sensor::ScanDataPtr<double>;

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
    virtual void Score(const GridMapType& gridMap,
                       const ScanPtr& scanData,
                       const RobotPose2D<double>& sensorPose,
                       Summary& resultSummary) = 0;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_SCORE_FUNCTION_HPP */
