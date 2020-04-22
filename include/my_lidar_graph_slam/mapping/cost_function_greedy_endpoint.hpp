
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

    /* Calculate a gradient vector */
    Eigen::Vector3d ComputeGradient(
        const GridMapType& gridMap,
        const ScanType& scanData,
        const RobotPose2D<double>& sensorPose) override;
    
    /* Calculate a covariance matrix */
    Eigen::Matrix3d ComputeCovariance(
        const GridMapType& gridMap,
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
        const double scanRange = scanData->RangeAt(i);
        
        if (scanRange >= maxRange || scanRange <= minRange)
            continue;
        
        /* Calculate the grid cell index corresponding to the
         * scan point and the missed point */
        Point2D<double> hitPoint;
        Point2D<double> missedPoint;
        scanData->HitAndMissedPoint(sensorPose, i, this->mHitAndMissedDist,
                                    hitPoint, missedPoint);
        
        const Point2D<int> hitPointIdx =
            gridMap.WorldCoordinateToGridCellIndex(hitPoint);
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
        
        /* Add to the cost value, which is proportional to the negative
         * log-likelihood of the observation probability and represents the
         * degree of the mismatch between the laser scan and the grid map */
        costValue += minSquaredDist;
    }

    /* Apply the scaling factor to the cost value */
    costValue *= this->mScalingFactor;

    return costValue;
}

/* Calculate a gradient vector */
template <typename T, typename U>
Eigen::Vector3d CostGreedyEndpoint<T, U>::ComputeGradient(
    const GridMapType& gridMap,
    const ScanType& scanData,
    const RobotPose2D<double>& sensorPose)
{
    /* Compute a gradient of the cost function with respect to sensor pose */
    const double diffLinear = 0.01;
    const double diffAngular = 0.01;

    const RobotPose2D<double> deltaX { diffLinear, 0.0, 0.0 };
    const RobotPose2D<double> deltaY { 0.0, diffLinear, 0.0 };
    const RobotPose2D<double> deltaTheta { 0.0, 0.0, diffAngular };

    const auto costValue = [&](const RobotPose2D<double>& pose) {
        return this->Cost(gridMap, scanData, pose); };
    
    const double diffCostX = costValue(sensorPose + deltaX) -
                             costValue(sensorPose - deltaX);
    const double diffCostY = costValue(sensorPose + deltaY) -
                             costValue(sensorPose - deltaY);
    const double diffCostTheta = costValue(sensorPose + deltaTheta) -
                                 costValue(sensorPose - deltaTheta);
    
    /* Calculate gradients */
    const double gradX = 0.5 * diffCostX / diffLinear;
    const double gradY = 0.5 * diffCostY / diffLinear;
    const double gradTheta = 0.5 * diffCostTheta / diffAngular;

    return Eigen::Vector3d { gradX, gradY, gradTheta };
}

/* Calculate a covariance matrix */
template <typename T, typename U>
Eigen::Matrix3d CostGreedyEndpoint<T, U>::ComputeCovariance(
    const GridMapType& gridMap,
    const ScanType& scanData,
    const RobotPose2D<double>& sensorPose)
{
    /* Approximate a covariance matrix using Laplace approximation
     * Covariance matrix is computed from the inverse of a Hessian matrix
     * of a cost function at the estimated robot pose (optimum point) */
    /* Hessian matrix is then calculated using a Jacobian matrix based on
     * Gauss-Newton approximation */

    /* Calculate the gradient vector */
    const Eigen::Vector3d gradVec =
        this->ComputeGradient(gridMap, scanData, sensorPose);

    /* Create a covariance matrix */
    Eigen::Matrix3d covMat = gradVec * gradVec.transpose();

    /* Add a small constant to the diagonal elements */
    covMat(0, 0) += 0.01;
    covMat(1, 1) += 0.01;
    covMat(2, 2) += 0.01;

    return covMat;
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_COST_FUNCTION_GREEDY_ENDPOINT_HPP */
