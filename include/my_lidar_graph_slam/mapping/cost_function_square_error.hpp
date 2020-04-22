
/* cost_function_square_error.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_COST_FUNCTION_SQUARE_ERROR_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_COST_FUNCTION_SQUARE_ERROR_HPP

#include "my_lidar_graph_slam/pose.hpp"
#include "my_lidar_graph_slam/mapping/cost_function.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

template <typename T, typename U>
class CostSquareError final : public CostFunction<T, U>
{
public:
    /* Type definitions */
    using typename CostFunction<T, U>::GridMapType;
    using typename CostFunction<T, U>::ScanType;

    /* Constructor */
    CostSquareError(double usableRangeMin,
                    double usableRangeMax) :
        CostFunction<T, U>(),
        mUsableRangeMin(usableRangeMin),
        mUsableRangeMax(usableRangeMax) { }
    
    /* Destructor */
    ~CostSquareError() = default;

    /* Calculate cost function (mismatch between scan data and map) */
    double Cost(const GridMapType& gridMap,
                const ScanType& scanData,
                const RobotPose2D<double>& sensorPose) override;

    /* Calculate a gradient of the cost function
     * with respect to the robot pose */
    Eigen::Vector3d ComputeGradient(
        const GridMapType& gridMap,
        const ScanType& scanData,
        const RobotPose2D<double>& sensorPose) override;
    
    /* Calculate a covariance matrix */
    Eigen::Matrix3d ComputeCovariance(
        const GridMapType& gridMap,
        const ScanType& scanData,
        const RobotPose2D<double>& sensorPose) override;
    
    /* Calculate a gradient of the cost function
     * with respect to the robot pose numerically */
    Eigen::Vector3d ComputeNumericalGradient(
        const GridMapType& gridMap,
        const ScanType& scanData,
        const RobotPose2D<double>& sensorPose);
    
    /* Calculate a gradient of the smoothed map function
     * with respect to the specified position on the map */
    Eigen::Vector2d ComputeMapGradient(const GridMapType& gridMap,
                                       const Point2D<double>& mapPos);
    
    /* Calculate a gradient of the smoothed map function
     * at the specified scan point with respect to the robot pose */
    Eigen::Vector3d ComputeMapGradient(const GridMapType& gridMap,
                                       const RobotPose2D<double>& sensorPose,
                                       const double scanRange,
                                       const double scanAngle);

    /* Calculate a gradient of the smoothed map function
     * at the specified scan point with respect to the robot pose */
    Eigen::Vector3d ComputeMapNumericalGradient(
        const GridMapType& gridMap,
        const RobotPose2D<double>& sensorPose,
        const double scanRange,
        const double scanAngle);
    
    /* Calculate the smoothed occupancy probability value
     * using bicubic interpolation */
    double ComputeSmoothedValue(
        const GridMapType& gridMap,
        const Point2D<double>& gridCellIdx) const;

private:
    /* Minimum laser scan range considered for calculation */
    double mUsableRangeMin;
    /* Maximum laser scan range considered for calculation */
    double mUsableRangeMax;
};

/* Calculate cost function based on the squared error of the
 * occupancy probability value */
template <typename T, typename U>
double CostSquareError<T, U>::Cost(const GridMapType& gridMap,
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
        /* Retrieve the scan range */
        const double scanRange = scanData->RangeAt(i);
        
        if (scanRange >= maxRange || scanRange <= minRange)
            continue;
        
        /* Calculate the grid cell index corresponding to the scan point */
        const Point2D<double> hitPoint = scanData->HitPoint(sensorPose, i);
        
        /* Calculate the smoothed occupancy probability value
         * this must be close to 1 since it corresponds to the hit point */
        const Point2D<double> floatIdx =
            gridMap.WorldCoordinateToGridCellIndexFloat(hitPoint);
        const double smoothedValue =
            this->ComputeSmoothedValue(gridMap, floatIdx);
        /* Calculate the squared error */
        const double squaredError = std::pow(1.0 - smoothedValue, 2.0);
        /* Add to the total cost value */
        costValue += squaredError;
    }

    return costValue;
}

/* Calculate a gradient vector */
template <typename T, typename U>
Eigen::Vector3d CostSquareError<T, U>::ComputeGradient(
    const GridMapType& gridMap,
    const ScanType& scanData,
    const RobotPose2D<double>& sensorPose)
{
    double gradX = 0.0;
    double gradY = 0.0;
    double gradTheta = 0.0;

    /* Compute a gradient of the cost function with respect to sensor pose */
    const double minRange = std::max(
        this->mUsableRangeMin, scanData->MinRange());
    const double maxRange = std::min(
        this->mUsableRangeMax, scanData->MaxRange());
    
    const std::size_t numOfScans = scanData->Ranges().size();

    for (std::size_t i = 0; i < numOfScans; ++i) {
        /* Retrieve the scan range and angle */
        const double scanRange = scanData->RangeAt(i);
        const double scanAngle = scanData->AngleAt(i);
        
        if (scanRange >= maxRange || scanRange <= minRange)
            continue;
        
        /* Calculate the grid cell index corresponding to the scan point */
        const Point2D<double> hitPoint = scanData->HitPoint(sensorPose, i);
        const Point2D<double> floatingIdx =
            gridMap.WorldCoordinateToGridCellIndexFloat(hitPoint);
        
        /* Calculate the smoothed occupancy probability value */
        const double smoothedMapValue =
            this->ComputeSmoothedValue(gridMap, floatingIdx);
        /* Calculate the error */
        const double errorValue = 1.0 - smoothedMapValue;

        /* Compute a gradient of the smoothed occupancy probability value
         * at the scan point with respect to the robot pose */
        const Eigen::Vector3d gradVec =
            this->ComputeMapGradient(gridMap, sensorPose, scanRange, scanAngle);

        /* Update gradients */
        gradX += 2.0 * errorValue * (-gradVec(0));
        gradY += 2.0 * errorValue * (-gradVec(1));
        gradTheta += 2.0 * errorValue * (-gradVec(2));
    }

    return Eigen::Vector3d { gradX, gradY, gradTheta };
}

/* Calculate a covariance matrix */
template <typename T, typename U>
Eigen::Matrix3d CostSquareError<T, U>::ComputeCovariance(
    const GridMapType& gridMap,
    const ScanType& scanData,
    const RobotPose2D<double>& sensorPose)
{
    /* Covariance matrix is calculated using Laplace approximation, which means
     * that the matrix is computed from the inverse of a Hessian matrix */
    /* Hessian matrix is calculated using a gradient of the cost function
     * with respect to the sensor pose */

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

/* Calculate a gradient vector numerically */
template <typename T, typename U>
Eigen::Vector3d CostSquareError<T, U>::ComputeNumericalGradient(
    const GridMapType& gridMap,
    const ScanType& scanData,
    const RobotPose2D<double>& sensorPose)
{
    /* Compute a gradient of the cost function with respect to sensor pose
     * by numerical gradient (for debugging purpose) */
    const double diffLinear = 1e-4;
    const double diffAngular = 1e-4;

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

/* Calculate a gradient of the smoothed map function
 * with respect to the specified position on the map */
template <typename T, typename U>
Eigen::Vector2d CostSquareError<T, U>::ComputeMapGradient(
    const GridMapType& gridMap,
    const Point2D<double>& mapPos)
{
    const double deltaIdx = 0.1;
    const double deltaDist = gridMap.MapResolution() * deltaIdx;
    const Point2D<double> deltaX { deltaIdx / 2.0, 0.0 };
    const Point2D<double> deltaY { 0.0, deltaIdx / 2.0 };

    const auto mapValue = [&](const Point2D<double>& floatingIdx) {
        return this->ComputeSmoothedValue(gridMap, floatingIdx); };
    
    /* Compute a floating-point grid cell index corresponding to the
     * specified position on the map */
    const Point2D<double> floatingGridCellIdx =
        gridMap.WorldCoordinateToGridCellIndexFloat(mapPos);
    
    /* Compute a gradient of the smoothed occupancy probability value
     * with respect to the map position */
    const double diffMapX = mapValue(floatingGridCellIdx + deltaX) -
                            mapValue(floatingGridCellIdx - deltaX);
    const double diffMapY = mapValue(floatingGridCellIdx + deltaY) -
                            mapValue(floatingGridCellIdx - deltaY);
    const double gradMapX = diffMapX / deltaDist;
    const double gradMapY = diffMapY / deltaDist;

    return Eigen::Vector2d { gradMapX, gradMapY };
}

/* Calculate a gradient of the smoothed map function
 * at the specified scan point with respect to the robot pose */
template <typename T, typename U>
Eigen::Vector3d CostSquareError<T, U>::ComputeMapGradient(
    const GridMapType& gridMap,
    const RobotPose2D<double>& sensorPose,
    const double scanRange,
    const double scanAngle)
{
    /* Calculate the grid cell index corresponding to the scan point */
    const double cosTheta = std::cos(sensorPose.mTheta + scanAngle);
    const double sinTheta = std::sin(sensorPose.mTheta + scanAngle);
    
    /* Compute a gradient of the smoothed occupancy probability value
     * with respect to the scan point */
    const Point2D<double> hitPoint {
        sensorPose.mX + scanRange * cosTheta,
        sensorPose.mY + scanRange * sinTheta };
    const Eigen::Vector2d gradVec =
        this->ComputeMapGradient(gridMap, hitPoint);
    
    /* Compute a gradient of the smoothed occupancy probability value
     * at the scan point with respect to the robot pose */
    const double gradX = gradVec(0);
    const double gradY = gradVec(1);
    const double gradTheta = -scanRange * sinTheta * gradVec(0) +
                              scanRange * cosTheta * gradVec(1);
    
    return Eigen::Vector3d { gradX, gradY, gradTheta };
}

/* Calculate a gradient of the smoothed map function
 * at the specified scan point with respect to the robot pose */
template <typename T, typename U>
Eigen::Vector3d CostSquareError<T, U>::ComputeMapNumericalGradient(
    const GridMapType& gridMap,
    const RobotPose2D<double>& sensorPose,
    const double scanRange,
    const double scanAngle)
{
    /* Compute a smoothed occupancy probability value
     * given a sensor pose and a scan data */
    const auto mapValue = [&](const RobotPose2D<double>& pose) {
        const double cosTheta = std::cos(pose.mTheta + scanAngle);
        const double sinTheta = std::sin(pose.mTheta + scanAngle);
        const Point2D<double> hitPoint {
            pose.mX + scanRange * cosTheta,
            pose.mY + scanRange * sinTheta };
        const Point2D<double> floatingIdx =
            gridMap.WorldCoordinateToGridCellIndexFloat(hitPoint);
        return this->ComputeSmoothedValue(gridMap, floatingIdx);
    };

    const double diffLinear = 1e-2;
    const double diffAngular = 1e-2;

    const RobotPose2D<double> deltaX { diffLinear, 0.0, 0.0 };
    const RobotPose2D<double> deltaY { 0.0, diffLinear, 0.0 };
    const RobotPose2D<double> deltaTheta { 0.0, 0.0, diffAngular };

    const double diffMapX = mapValue(sensorPose + deltaX) -
                            mapValue(sensorPose - deltaX);
    const double diffMapY = mapValue(sensorPose + deltaY) -
                            mapValue(sensorPose - deltaY);
    const double diffMapTheta = mapValue(sensorPose + deltaTheta) -
                                mapValue(sensorPose - deltaTheta);

    /* Calculate gradients */
    const double gradX = 0.5 * diffMapX / diffLinear;
    const double gradY = 0.5 * diffMapY / diffLinear;
    const double gradTheta = 0.5 * diffMapTheta / diffAngular;

    return Eigen::Vector3d { gradX, gradY, gradTheta };
}

/* Calculate the smoothed occupancy probability value
 * using bicubic interpolation */
template <typename T, typename U>
double CostSquareError<T, U>::ComputeSmoothedValue(
    const GridMapType& gridMap,
    const Point2D<double>& gridCellIdx) const
{
    /* Interpolation kernel function */
    auto h = [](double t) -> double {
        const double at = std::abs(t);

        if (at <= 1.0) {
            const double at3 = std::pow(at, 3.0);
            const double at2 = std::pow(at, 2.0);
            return (at3 - 2.0 * at2 + 1.0);
        } else if (at <= 2.0) {
            const double at3 = std::pow(at, 3.0);
            const double at2 = std::pow(at, 2.0);
            return (-at3 + 5.0 * at2 - 8.0 * at + 4.0);
        }
        
        return 0.0;
    };

    /* Occupancy grid value function */
    auto f = [&gridMap](double x, double y) -> double {
        /* If the specified grid cell index is out of bounds,
         * the value of the first/last row/column grid cell is returned */
        const int xc = std::clamp(static_cast<int>(x),
                                  0, gridMap.NumOfGridCellsX() - 1);
        const int yc = std::clamp(static_cast<int>(y),
                                  0, gridMap.NumOfGridCellsY() - 1);

        /* Default value is returned if the grid cell is not yet allocated */
        /* If the grid cell is allocated but not yet observed,
         * the unknown value GridCell::Unknown (0) is returned */
        return gridMap.Value(xc, yc, 0.0);
    };

    /* Perform bicubic interpolation */
    const double x = gridCellIdx.mX;
    const double y = gridCellIdx.mY;
    const double floorX = std::floor(x);
    const double floorY = std::floor(y);

    const double x1 = 1.0 + x - floorX;
    const double x2 = x - floorX;
    const double x3 = floorX + 1.0 - x;
    const double x4 = floorX + 2.0 - x;

    const double y1 = 1.0 + y - floorY;
    const double y2 = y - floorY;
    const double y3 = floorY + 1.0 - y;
    const double y4 = floorY + 2.0 - y;

    const Eigen::Vector4d vecX { h(x1), h(x2), h(x3), h(x4) };
    const Eigen::Vector4d vecY { h(y1), h(y2), h(y3), h(y4) };

    Eigen::Matrix4d matValue;
    matValue << f(x - x1, y - y1), f(x - x1, y - y2),
                f(x - x1, y + y3), f(x - x1, y + y4),
                f(x - x2, y - y1), f(x - x2, y - y2),
                f(x - x2, y + y3), f(x - x2, y + y4),
                f(x + x3, y - y1), f(x + x3, y - y2),
                f(x + x3, y + y3), f(x + x3, y + y4),
                f(x + x4, y - y1), f(x + x4, y - y2),
                f(x + x4, y + y3), f(x + x4, y + y4);
    
    /* Compute the smoothed value */
    const double smoothedValue = vecX.transpose() * matValue * vecY;

    /* Clamp the occupancy value */
    return std::clamp(smoothedValue, 0.0, 1.0);
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_COST_FUNCTION_SQUARE_ERROR_HPP */
