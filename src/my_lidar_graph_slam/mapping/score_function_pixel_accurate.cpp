
/* score_function_pixel_accurate.cpp */

#include "my_lidar_graph_slam/mapping/score_function_pixel_accurate.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/* Constructor */
ScorePixelAccurate::ScorePixelAccurate(
    double usableRangeMin,
    double usableRangeMax) :
    mUsableRangeMin(usableRangeMin),
    mUsableRangeMax(usableRangeMax)
{
}

/* Evaluate score function (matching score between scan data and map) */
void ScorePixelAccurate::Score(
    const GridMapBase<double>& gridMap,
    const Sensor::ScanDataPtr<double>& scanData,
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

    const double unknownVal = gridMap.UnknownValue();

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
            gridMap.Value(hitPointIdx, unknownVal);
        
        /* Count the grid cells with valid occupancy probability value */
        if (hitGridCellValue != unknownVal)
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
