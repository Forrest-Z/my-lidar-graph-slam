
/* score_function_pixel_accurate.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_SCORE_FUNCTION_PIXEL_ACCURATE_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_SCORE_FUNCTION_PIXEL_ACCURATE_HPP

#include "my_lidar_graph_slam/mapping/score_function.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

class ScorePixelAccurate final : public ScoreFunction
{
public:
    /* Type definitions */
    using ScoreFunction::GridMapType;
    using ScoreFunction::ScanPtr;

    /* Constructor */
    ScorePixelAccurate(double usableRangeMin,
                       double usableRangeMax) :
        mUsableRangeMin(usableRangeMin),
        mUsableRangeMax(usableRangeMax) { }
    
    /* Destructor */
    ~ScorePixelAccurate() = default;

    /* Evaluate score function (matching score between scan data and map) */
    void Score(const GridMapType& gridMap,
               const ScanPtr& scanData,
               const RobotPose2D<double>& sensorPose,
               Summary& resultSummary) override;

private:
    /* Minimum laser scan range considered for calculation */
    double mUsableRangeMin;
    /* Maximum laser scan range considered for calculation */
    double mUsableRangeMax;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_SCORE_FUNCTION_PIXEL_ACCURATE_HPP */
