
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
void ScanMatcherHillClimbing::OptimizePose(
    const GridMapBase<double>& gridMap,
    const Sensor::ScanDataPtr<double>& scanData,
    const RobotPose2D<double>& initialPose,
    Summary& resultSummary)
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
    
    /* Calculate the pose covariance matrix */
    const Eigen::Matrix3d covMat = this->mCostFunc->ComputeCovariance(
        gridMap, scanData, bestPose);

    /* Calculate the robot pose from the updated sensor pose */
    const RobotPose2D<double> robotPose = MoveBackward(bestPose, relPose);
    
    /* Setup the result summary */
    /* Set the normalized cost value */
    resultSummary.mNormalizedCost = minCost / scanData->NumOfScans();
    /* Set the estimated robot pose */
    resultSummary.mEstimatedPose = robotPose;
    /* Set the estimated pose covariance matrix */
    resultSummary.mEstimatedCovariance = covMat;

    return;
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
