
/* scan_matcher_hill_climbing.cpp */

#include "my_lidar_graph_slam/mapping/scan_matcher_hill_climbing.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/* Constructor */
ScanMatcherHillClimbing::ScanMatcherHillClimbing(
    double linearStep,
    double angularStep,
    int maxIterations,
    int maxNumOfRefinements,
    const CostFuncPtr& costFunc) :
    ScanMatcher(),
    mLinearStep(linearStep),
    mAngularStep(angularStep),
    mMaxIterations(maxIterations),
    mMaxNumOfRefinements(maxNumOfRefinements),
    mCostFunc(costFunc)
{
}

/* Optimize the robot pose by scan matching */
ScanMatchingSummary ScanMatcherHillClimbing::OptimizePose(
    const ScanMatchingQuery& queryInfo)
{
    /* Constants used for hill-climbing method */
    static const double moveX[] = { 1.0, -1.0, 0.0, 0.0, 0.0, 0.0 };
    static const double moveY[] = { 0.0, 0.0, 1.0, -1.0, 0.0, 0.0 };
    static const double moveTheta[] = { 0.0, 0.0, 0.0, 0.0, 1.0, -1.0 };

    /* Retrieve the query information */
    const auto& gridMap = queryInfo.mGridMap;
    const auto& scanData = queryInfo.mScanData;
    const RobotPose2D<double>& initialPose = queryInfo.mInitialPose;

    /* Calculate the sensor pose from the initial robot pose */
    const RobotPose2D<double>& relPose = scanData->RelativeSensorPose();
    const RobotPose2D<double> sensorPose = Compound(initialPose, relPose);

    /* Minimum cost and the pose */
    double minCost = this->mCostFunc->Cost(gridMap, scanData, sensorPose);
    RobotPose2D<double> bestPose = sensorPose;

    /* The number of iterations */
    int numOfIterations = 0;
    /* The number of refinements (step parameter updates) */
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

    /* Compute the normalized cost value */
    const double normalizedCost = minCost / scanData->NumOfScans();
    /* Compute the estimated robot pose in a world frame */
    const RobotPose2D<double> estimatedPose =
        MoveBackward(bestPose, relPose);
    /* Calculate the pose covariance matrix in a world frame */
    const Eigen::Matrix3d estimatedCovariance =
        this->mCostFunc->ComputeCovariance(gridMap, scanData, bestPose);

    /* Return the normalized cost value, the estimated robot pose,
     * and the estimated pose covariance matrix in a world frame */
    return ScanMatchingSummary {
        true, normalizedCost, initialPose,
        estimatedPose, estimatedCovariance };
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
