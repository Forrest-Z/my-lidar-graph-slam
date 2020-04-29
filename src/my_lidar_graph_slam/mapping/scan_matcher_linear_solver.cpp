
/* scan_matcher_linear_solver.cpp */

#include "my_lidar_graph_slam/mapping/scan_matcher_linear_solver.hpp"

#include <cassert>

#include <Eigen/Core>
#include <Eigen/Dense>

namespace MyLidarGraphSlam {
namespace Mapping {

/* Constructor */
ScanMatcherLinearSolver::ScanMatcherLinearSolver(
    int numOfIterationsMax,
    double convergenceThreshold,
    double usableRangeMin,
    double usableRangeMax,
    double translationRegularizer,
    double rotationRegularizer,
    const CostFuncPtr& costFunc) :
    ScanMatcher(),
    mNumOfIterationsMax(numOfIterationsMax),
    mConvergenceThreshold(convergenceThreshold),
    mUsableRangeMin(usableRangeMin),
    mUsableRangeMax(usableRangeMax),
    mTranslationRegularizer(translationRegularizer),
    mRotationRegularizer(rotationRegularizer),
    mCostFunc(nullptr)
{
    /* Ensure that the cost function is evaluated based on square error */
    this->mCostFunc = std::dynamic_pointer_cast<CostSquareError>(costFunc);
    assert(this->mCostFunc != nullptr);
}

/* Optimize the robot pose by scan matching */
void ScanMatcherLinearSolver::OptimizePose(
    const GridMapType& gridMap,
    const Sensor::ScanDataPtr<double>& scanData,
    const RobotPose2D<double>& initialPose,
    Summary& resultSummary)
{
    /* Calculate the sensor pose from the initial robot pose */
    const RobotPose2D<double>& relPose = scanData->RelativeSensorPose();
    RobotPose2D<double> sensorPose = Compound(initialPose, relPose);

    /* Minimum cost and the pose */
    double prevCost = std::numeric_limits<double>::max();
    double cost = std::numeric_limits<double>::max();
    int numOfIterations = 0;
    
    while (true) {
        /* Perform one scan matching step */
        sensorPose = this->OptimizeStep(gridMap, scanData, sensorPose);
        /* Compute the cost value */
        cost = this->mCostFunc->Cost(gridMap, scanData, sensorPose);

        /* Stop the optimization if the number of iteration steps
         * exceeded the maximum or the cost converged */
        if (++numOfIterations >= this->mNumOfIterationsMax ||
            std::fabs(prevCost - cost) < this->mConvergenceThreshold)
            break;

        prevCost = cost;
    }

    /* Calculate the pose covariance matrix */
    const Eigen::Matrix3d covMat = this->mCostFunc->ComputeCovariance(
        gridMap, scanData, sensorPose);

    /* Calculate the robot pose from the updated sensor pose */
    const RobotPose2D<double> robotPose = MoveBackward(sensorPose, relPose);

    /* Setup the result summary */
    /* Set the normalized cost value */
    resultSummary.mNormalizedCost = cost / scanData->NumOfScans();
    /* Set the estimated robot pose */
    resultSummary.mEstimatedPose = robotPose;
    /* Set the estimated pose covariance matrix */
    resultSummary.mEstimatedCovariance = covMat;

    return;
}

/* Perform one optimization step */
RobotPose2D<double> ScanMatcherLinearSolver::OptimizeStep(
    const GridMapType& gridMap,
    const Sensor::ScanDataPtr<double>& scanData,
    const RobotPose2D<double>& sensorPose)
{
    /* Construct and compute the linear system */
    Eigen::Vector3d vecB = Eigen::Vector3d::Zero();
    Eigen::Matrix3d matH = Eigen::Matrix3d::Zero();

    const double minRange = std::max(
        this->mUsableRangeMin, scanData->MinRange());
    const double maxRange = std::min(
        this->mUsableRangeMax, scanData->MaxRange());
    
    const std::size_t numOfScans = scanData->NumOfScans();

    /* Setup a dense matrix and a vector for coefficients */
    for (std::size_t i = 0; i < numOfScans; ++i) {
        /* Retrieve the scan range and angle */
        const double scanRange = scanData->RangeAt(i);
        const double scanAngle = scanData->AngleAt(i);
        
        if (scanRange >= maxRange || scanRange <= minRange)
            continue;
        
        /* Calculate the hit point */
        const Point2D<double> hitPoint = scanData->HitPoint(sensorPose, i);

        /* Calculate the smoothed occupancy probability value */
        const Point2D<double> floatingIdx = 
            gridMap.WorldCoordinateToGridCellIndexFloat(hitPoint);
        const double smoothedMapValue =
            this->mCostFunc->ComputeSmoothedValue(gridMap, floatingIdx);
        /* Calculate the residual */
        const double residualValue = 1.0 - smoothedMapValue;
        
        /* Calculate a gradient of the smoothed map function
         * at the scan point with respect to the sensor pose */
        const Eigen::Vector3d gradVec =
            this->mCostFunc->ComputeMapGradient(
                gridMap, sensorPose, scanRange, scanAngle);
        
        /* Update the coefficients */
        vecB += residualValue * gradVec;
        matH += gradVec * gradVec.transpose();
    }

    /* Add the L2 regularization factor */
    matH(0, 0) += this->mTranslationRegularizer;
    matH(1, 1) += this->mTranslationRegularizer;
    matH(2, 2) += this->mRotationRegularizer;

    /* Solve the linear system using QR decomposition
     * and obtain the sensor pose increments */
    const Eigen::Vector3d vecDelta = matH.colPivHouseholderQr().solve(vecB);

    /* Update and return the sensor pose */
    return RobotPose2D<double> { sensorPose.mX + vecDelta(0),
                                 sensorPose.mY + vecDelta(1),
                                 sensorPose.mTheta + vecDelta(2) };
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
