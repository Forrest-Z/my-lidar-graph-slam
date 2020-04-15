
/* cost_function_greedy_endpoint.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_COST_FUNCTION_GREEDY_ENDPOINT_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_COST_FUNCTION_GREEDY_ENDPOINT_HPP

#include "my_lidar_graph_slam/pose.hpp"
#include "my_lidar_graph_slam/mapping/cost_function.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

template <typename T, typename U>
class CostGreedyEndpoint final : public CostFunction<T, U>
{
public:
    /* Type definitions */
    using typename CostFunction<T, U>::GridMapType;
    using typename CostFunction<T, U>::ScanType;

    /* Constructor */
    CostGreedyEndpoint(double usableRangeMin,
                       double usableRangeMax,
                       double hitAndMissedDist,
                       double occupancyThreshold,
                       double gaussianSigma,
                       int kernelSize,
                       double scalingFactor) :
        CostFunction<T, U>(),
        mUsableRangeMin(usableRangeMin),
        mUsableRangeMax(usableRangeMax),
        mHitAndMissedDist(hitAndMissedDist),
        mOccupancyThreshold(occupancyThreshold),
        mGaussianSigma(gaussianSigma),
        mKernelSize(kernelSize),
        mScalingFactor(scalingFactor) { }
    
    /* Destructor */
    ~CostGreedyEndpoint() = default;

    /* Calculate cost function (mismatch between scan data and map) */
    double Cost(const GridMapType& gridMap,
                const ScanType& scanData,
                const RobotPose2D<double>& sensorPose) override;

private:
    /* Minimum laser scan range considered for calculation */
    double mUsableRangeMin;
    /* Maximum laser scan range considered for calculation */
    double mUsableRangeMax;
    /* Distance between hit and missed grid cells */
    double mHitAndMissedDist;
    /* Probability threshold for being obstructed */
    double mOccupancyThreshold;
    /* Normal deviation value for Gaussian distribution */
    double mGaussianSigma;
    /* Size of searching window (in the number of grid cells) */
    int    mKernelSize;
    /* Scaling factor for cost value */
    double mScalingFactor;
};

/* Calculate cost function based on the squared distance
 * between scan point and its corresponding grid cell */
template <typename T, typename U>
double CostGreedyEndpoint<T, U>::Cost(const GridMapType& gridMap,
                                      const ScanType& scanData,
                                      const RobotPose2D<double>& sensorPose)
{
    double costValue = 0.0;

    const double minRange = std::max(
        this->mUsableRangeMin, scanData->MinRange());
    const double maxRange = std::min(
        this->mUsableRangeMax, scanData->MaxRange());
    
    const std::size_t numOfScans = scanData->Ranges().size();

    for (std::size_t i = 0; i < numOfScans; ++i) {
        const double range = scanData->Ranges().at(i);
        const double angle = scanData->MinAngle() +
            static_cast<double>(i) * scanData->AngleIncrement();
        
        if (range >= maxRange || range <= minRange)
            continue;
        
        /* Calculate the grid cell index corresponding to the scan point */
        const double cosTheta = std::cos(sensorPose.mTheta + angle);
        const double sinTheta = std::sin(sensorPose.mTheta + angle);

        const Point2D<double> hitPoint {
            sensorPose.mX + range * cosTheta,
            sensorPose.mY + range * sinTheta };
        const Point2D<int> hitPointIdx =
            gridMap.WorldCoordinateToGridCellIndex(hitPoint);
        
        /* Calculate the index for missed grid cell */
        const Point2D<double> missedPoint {
            hitPoint.mX - this->mHitAndMissedDist * cosTheta,
            hitPoint.mY - this->mHitAndMissedDist * sinTheta };
        const Point2D<int> missedPointIdx =
            gridMap.WorldCoordinateToGridCellIndex(missedPoint);
        
        /* Find the best grid cell index from the searching window */
        double minSquaredDist = gridMap.SquaredDistance(
            Point2D<int>(0, 0),
            Point2D<int>(this->mKernelSize + 1, this->mKernelSize + 1));

        for (int ky = -this->mKernelSize; ky <= this->mKernelSize; ++ky) {
            for (int kx = -this->mKernelSize; kx <= this->mKernelSize; ++kx) {
                const Point2D<int> hitIdx =
                    hitPointIdx + Point2D<int>(kx, ky);
                const typename T::ValueType hitCellValue =
                    gridMap.Value(hitIdx, T::GridCellType::Unknown);
                
                const Point2D<int> missedIdx =
                    missedPointIdx + Point2D<int>(kx, ky);
                const typename T::ValueType missedCellValue =
                    gridMap.Value(missedIdx, T::GridCellType::Unknown);
                
                /* Skip if the grid cell has unknown occupancy probability */
                if (hitCellValue == T::GridCellType::Unknown ||
                    missedCellValue == T::GridCellType::Unknown)
                    continue;
                
                /* Skip if the occupancy probability of the grid cell
                 * that is assumed to be hit is less than the threshold or
                 * the occupancy probability of the missed grid cell
                 * is greater than the threshold */
                if (hitCellValue < this->mOccupancyThreshold ||
                    missedCellValue > this->mOccupancyThreshold)
                    continue;
                
                /* Calculate the distance between two grid cells */
                const double squaredDist =
                    gridMap.SquaredDistance(hitPointIdx, hitIdx);
                minSquaredDist = std::min(squaredDist, minSquaredDist);
            }
        }

        /* Add to the cost value, which is the negative log-likelihood
         * of the observation probability and represents the degree of
         * mismatch between the laser scan and the grid map */
        costValue += (minSquaredDist / (0.5 * this->mGaussianSigma));
    }

    /* Apply the scaling factor to the cost value */
    costValue *= this->mScalingFactor;

    return costValue;
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_COST_FUNCTION_GREEDY_ENDPOINT_HPP */
