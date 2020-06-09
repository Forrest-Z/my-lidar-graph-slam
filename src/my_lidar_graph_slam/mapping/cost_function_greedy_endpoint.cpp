
/* cost_function_greedy_endpoint.cpp */

#include "my_lidar_graph_slam/mapping/cost_function_greedy_endpoint.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/* Constructor */
CostGreedyEndpoint::CostGreedyEndpoint(
    double usableRangeMin,
    double usableRangeMax,
    double hitAndMissedDist,
    double occupancyThreshold,
    int kernelSize,
    double scalingFactor,
    double standardDeviation) :
    CostFunction(),
    mUsableRangeMin(usableRangeMin),
    mUsableRangeMax(usableRangeMax),
    mHitAndMissedDist(hitAndMissedDist),
    mOccupancyThreshold(occupancyThreshold),
    mKernelSize(kernelSize),
    mStandardDeviation(standardDeviation),
    mVariance(standardDeviation * standardDeviation),
    mScalingFactor(scalingFactor)
{
}

/* Calculate cost function based on the squared distance
 * between scan point and its corresponding grid cell */
double CostGreedyEndpoint::Cost(
    const GridMapBase<double>& gridMap,
    const Sensor::ScanDataPtr<double>& scanData,
    const RobotPose2D<double>& sensorPose)
{
    double costValue = 0.0;

    const double minRange = std::max(
        this->mUsableRangeMin, scanData->MinRange());
    const double maxRange = std::min(
        this->mUsableRangeMax, scanData->MaxRange());
    
    const std::size_t numOfScans = scanData->NumOfScans();

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
        const double unknownVal = gridMap.UnknownValue();
        double minSquaredDist = gridMap.SquaredDistance(
            0, 0, this->mKernelSize + 1, this->mKernelSize + 1);

        for (int ky = -this->mKernelSize; ky <= this->mKernelSize; ++ky) {
            for (int kx = -this->mKernelSize; kx <= this->mKernelSize; ++kx) {
                const Point2D<int> hitIdx {
                    hitPointIdx.mX + kx, hitPointIdx.mY + ky };
                const double hitCellValue =
                    gridMap.Value(hitIdx, unknownVal);
                
                const Point2D<int> missedIdx {
                    missedPointIdx.mX + kx, missedPointIdx.mY + ky };
                const double missedCellValue =
                    gridMap.Value(missedIdx, unknownVal);
                
                /* Skip if the grid cell has unknown occupancy probability */
                if (hitCellValue == unknownVal ||
                    missedCellValue == unknownVal)
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
        costValue -= std::exp(-0.5 * minSquaredDist / this->mVariance);
    }

    /* Apply the scaling factor to the cost value */
    costValue *= this->mScalingFactor;

    return costValue;
}

/* Calculate a gradient vector */
Eigen::Vector3d CostGreedyEndpoint::ComputeGradient(
    const GridMapBase<double>& gridMap,
    const Sensor::ScanDataPtr<double>& scanData,
    const RobotPose2D<double>& sensorPose)
{
    /* Compute the step */
    const double diffLinear = gridMap.Resolution();
    const double diffAngular = 1e-2;

    /* Compute a gradient of the cost function with respect to sensor pose */
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
Eigen::Matrix3d CostGreedyEndpoint::ComputeCovariance(
    const GridMapBase<double>& gridMap,
    const Sensor::ScanDataPtr<double>& scanData,
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
