
/* score_function_pixel_accurate.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_SCORE_FUNCTION_PIXEL_ACCURATE_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_SCORE_FUNCTION_PIXEL_ACCURATE_HPP

#include "my_lidar_graph_slam/mapping/score_function.hpp"

#include "my_lidar_graph_slam/point.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

template <typename T, typename U>
class ScorePixelAccurate final : public ScoreFunction<T, U>
{
public:
    /* Type definitions */
    using typename ScoreFunction<T, U>::GridMapType;
    using typename ScoreFunction<T, U>::ScanPtr;
    using typename ScoreFunction<T, U>::Summary;

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

/* Evaluate score function (matching score between scan data and map) */
template <typename T, typename U>
void ScorePixelAccurate<T, U>::Score(const GridMapType& gridMap,
                                     const ScanPtr& scanData,
                                     const RobotPose2D<double>& sensorPose,
                                     Summary& resultSummary)
{
    double sumScore = 0.0;

    const double minRange = std::max(
        this->mUsableRangeMin, scanData->MinRange());
    const double maxRange = std::min(
        this->mUsableRangeMax, scanData->MaxRange());
    
    const std::size_t numOfScans = scanData->NumOfScans();
    std::size_t numOfValidGridCells = 0;

    for (std::size_t i = 0; i < numOfScans; ++i) {
        const double scanRange = scanData->RangeAt(i);

        if (scanRange >= maxRange || scanRange <= minRange)
            continue;
        
        /* Add the occupancy probability value at the hit point */
        const Point2D<double> hitPoint =
            scanData->HitPoint(sensorPose, i);
        const Point2D<int> hitPointIdx =
            gridMap.WorldCoordinateToGridCellIndex(hitPoint);
        const double hitGridCellValue =
            gridMap.Value(hitPointIdx, GridMapType::GridCellType::Unknown);
        
        /* Count the grid cells with valid occupancy probability value */
        if (hitGridCellValue != GridMapType::GridCellType::Unknown)
            ++numOfValidGridCells;
        
        /* If the hit grid cell has unknown probability value,
         * the minimum score (zero) is added */
        sumScore += hitGridCellValue;
    }

    /* Normalize the score function */
    const double normalizedScore =
        sumScore / static_cast<double>(scanData->NumOfScans());
    
    /* Calculate the rate of valid grid cells */
    const double matchRate =
        static_cast<double>(numOfValidGridCells) /
        static_cast<double>(numOfScans);

    /* Set the result summary */
    resultSummary.mScore = sumScore;
    resultSummary.mNormalizedScore = normalizedScore;
    resultSummary.mMatchRate = matchRate;

    return;
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_SCORE_FUNCTION_PIXEL_ACCURATE_HPP */
