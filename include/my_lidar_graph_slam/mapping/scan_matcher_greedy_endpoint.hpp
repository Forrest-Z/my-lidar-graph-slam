
/* scan_matcher_greedy_endpoint.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_SCAN_MATCHER_GREEDY_ENDPOINT_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_SCAN_MATCHER_GREEDY_ENDPOINT_HPP

#include <memory>
#include <utility>

#include "my_lidar_graph_slam/util.hpp"
#include "my_lidar_graph_slam/mapping/cost_function.hpp"
#include "my_lidar_graph_slam/mapping/scan_matcher.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

template <typename T, typename U>
class ScanMatcherGreedyEndpoint final : public ScanMatcher<T, U>
{
public:
    /* Type definitions */
    using CostType = std::unique_ptr<CostFunction<T, U>>;

    using typename ScanMatcher<T, U>::GridMapType;
    using typename ScanMatcher<T, U>::ScanType;

    /* Constructor */
    ScanMatcherGreedyEndpoint(double linearStep,
                              double angularStep,
                              int maxIterations,
                              int maxNumOfRefinements,
                              CostType&& costFunc) :
        ScanMatcher<T, U>(),
        mLinearStep(linearStep),
        mAngularStep(angularStep),
        mMaxIterations(maxIterations),
        mMaxNumOfRefinements(maxNumOfRefinements),
        mCostFunc(std::move(costFunc)) { }
    
    /* Destructor */
    ~ScanMatcherGreedyEndpoint() = default;

    /* Optimize the robot pose by scan matching */
    void OptimizePose(const GridMapType& gridMap,
                      const ScanType& scanData,
                      const RobotPose2D<double>& initialPose,
                      RobotPose2D<double>& estimatedPose) override;
    
    /* Calculate a covariance matrix */
    void ComputeCovariance(const GridMapType& gridMap,
                           const ScanType& scanData,
                           const RobotPose2D<double>& robotPose,
                           Eigen::Matrix3d& estimatedCovMat) override;
    
private:
    /* Initial step of the linear components (x and y) */
    double   mLinearStep;
    /* Initial step of the angular component (theta) */
    double   mAngularStep;
    /* Maximum number of iterations */
    int      mMaxIterations;
    /* Maximum number of step parameter updates */
    int      mMaxNumOfRefinements;
    /* Cost function */
    CostType mCostFunc;
};

/* Optimize the robot pose by scan matching */
template <typename T, typename U>
void ScanMatcherGreedyEndpoint<T, U>::OptimizePose(
    const GridMapType& gridMap,
    const ScanType& scanData,
    const RobotPose2D<double>& initialPose,
    RobotPose2D<double>& estimatedPose)
{
    /* Constants used for hill-climbing method */
    static const double moveX[] = { 1.0, -1.0, 0.0, 0.0, 0.0, 0.0 };
    static const double moveY[] = { 0.0, 0.0, 1.0, -1.0, 0.0, 0.0 };
    static const double moveTheta[] = { 0.0, 0.0, 0.0, 0.0, 1.0, -1.0 };

    /* Calculate the sensor pose from the initial robot pose */
    const RobotPose2D<double>& relPose = scanData->RelativeSensorPose();
    RobotPose2D<double> sensorPose = Compound(initialPose, relPose);

    /* Minimum cost and the pose */
    double minCost = this->mCostFunc->Cost(gridMap, scanData, sensorPose);
    RobotPose2D<double> bestPose = sensorPose;

    /* The number of the refinements (step parameter updates) */
    int numOfIterations = 0;
    int numOfRefinements = 0;
    double currentLinearStep = this->mLinearStep;
    double currentAngularStep = this->mAngularStep;
    bool poseUpdated = false;

    do {
        /* Local optimization */
        double minLocalCost = minCost;
        RobotPose2D<double> bestLocalPose = bestPose;
        poseUpdated = false;

        for (int i = 0; i < 6; ++i) {
            /* Move forward, backward, left, right,
             * rotate left and rotate right a little bit
             * then calculate the cost value */
            RobotPose2D<double> localPose = bestPose;
            localPose.mX += moveX[i] * currentLinearStep;
            localPose.mY += moveY[i] * currentLinearStep;
            localPose.mTheta += moveTheta[i] * currentAngularStep;

            /* Calculate the cost */
            const double localCost =
                this->mCostFunc->Cost(gridMap, scanData, localPose);

            if (localCost < minLocalCost) {
                minLocalCost = localCost;
                bestLocalPose = localPose;
                poseUpdated = true;
            }
        }

        /* Update best pose */
        if (poseUpdated) {
            minCost = minLocalCost;
            bestPose = bestLocalPose;
        } else {
            /* Update the step value if pose not improved */
            ++numOfRefinements;
            currentLinearStep *= 0.5;
            currentAngularStep *= 0.5;
        }
    } while ((poseUpdated || numOfRefinements < this->mMaxNumOfRefinements) &&
             (++numOfIterations < this->mMaxIterations));
    
    /* Calculate the robot pose from the updated sensor pose */
    RobotPose2D<double> robotPose = MoveBackward(bestPose, relPose);
    
    /* Set the estimated robot pose */
    estimatedPose = robotPose;

    return;
}

/* Calculate a covariance matrix */
template <typename T, typename U>
void ScanMatcherGreedyEndpoint<T, U>::ComputeCovariance(
    const GridMapType& gridMap,
    const ScanType& scanData,
    const RobotPose2D<double>& robotPose,
    Eigen::Matrix3d& estimatedCovMat)
{
    /* Calculate the sensor pose from the robot pose */
    const RobotPose2D<double>& relPose = scanData->RelativeSensorPose();
    const RobotPose2D<double> sensorPose = Compound(robotPose, relPose);

    /* Approximate a covariance matrix using Laplace approximation
     * Covariance matrix is computed from the inverse of a Hessian matrix
     * of a cost function at the estimated robot pose (optimum point) */
    /* Hessian matrix is then calculated using a Jacobian matrix based on
     * Gauss-Newton approximation */

    /* Calculate the cost function */
    const double diffLinear = 0.01;
    const double diffAngular = 0.01;

    const RobotPose2D<double> deltaX { diffLinear, 0.0, 0.0 };
    const RobotPose2D<double> deltaY { 0.0, diffLinear, 0.0 };
    const RobotPose2D<double> deltaTheta { 0.0, 0.0, diffAngular };

    const auto costValue = [&](const RobotPose2D<double>& pose) {
        return this->mCostFunc->Cost(gridMap, scanData, pose); };

    const double costX0 = costValue(sensorPose - deltaX);
    const double costX1 = costValue(sensorPose + deltaX);
    const double costY0 = costValue(sensorPose - deltaY);
    const double costY1 = costValue(sensorPose + deltaY);
    const double costTheta0 = costValue(sensorPose - deltaTheta);
    const double costTheta1 = costValue(sensorPose + deltaTheta);

    /* Calculate a partial derivatives */
    const double partialX = 0.5 * (costX1 - costX0) / diffLinear;
    const double partialY = 0.5 * (costY1 - costY0) / diffLinear;
    const double partialTheta = 0.5 * (costTheta1 - costTheta0) / diffAngular;

    /* Create a covariance matrix */
    estimatedCovMat(0, 0) = partialX * partialX;
    estimatedCovMat(0, 1) = partialX * partialY;
    estimatedCovMat(0, 2) = partialX * partialTheta;
    estimatedCovMat(1, 0) = estimatedCovMat(0, 1);
    estimatedCovMat(1, 1) = partialY * partialY;
    estimatedCovMat(1, 2) = partialY * partialTheta;
    estimatedCovMat(2, 0) = estimatedCovMat(0, 2);
    estimatedCovMat(2, 1) = estimatedCovMat(1, 2);
    estimatedCovMat(2, 2) = partialTheta * partialTheta;

    /* Add a small constant to the diagonal elements */
    estimatedCovMat(0, 0) += 0.01;
    estimatedCovMat(1, 1) += 0.01;
    estimatedCovMat(2, 2) += 0.01;

    return;
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_SCAN_MATCHER_GREEDY_ENDPOINT_HPP */
