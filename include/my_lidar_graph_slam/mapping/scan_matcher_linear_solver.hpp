
/* scan_matcher_linear_solver.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_SCAN_MATCHER_LINEAR_SOLVER_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_SCAN_MATCHER_LINEAR_SOLVER_HPP

#include "my_lidar_graph_slam/mapping/scan_matcher.hpp"

#include "my_lidar_graph_slam/mapping/cost_function.hpp"
#include "my_lidar_graph_slam/mapping/cost_function_square_error.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

class ScanMatcherLinearSolver final : public ScanMatcher
{
public:
    /* Constructor */
    ScanMatcherLinearSolver(int numOfIterationsMax,
                            double convergenceThreshold,
                            double usableRangeMin,
                            double usableRangeMax,
                            double translationRegularizer,
                            double rotationRegularizer,
                            const CostFuncPtr& costFunc);

    /* Destructor */
    ~ScanMatcherLinearSolver() = default;

    /* Optimize the robot pose by scan matching */
    ScanMatchingSummary OptimizePose(
        const ScanMatchingQuery& queryInfo) override;

private:
    /* Perform one optimization step */
    RobotPose2D<double> OptimizeStep(
        const GridMapType& gridMap,
        const Sensor::ScanDataPtr<double>& scanData,
        const RobotPose2D<double>& sensorPose);

private:
    /* Maximum number of the optimization iterations */
    int                mNumOfIterationsMax;
    /* Threshold to check the convergence */
    double             mConvergenceThreshold;
    /* Minimum laser scan range considered for calculation */
    double             mUsableRangeMin;
    /* Maximum laser scan range considered for calculation */
    double             mUsableRangeMax;
    /* Penalty for the translation in the scan matching process */
    double             mTranslationRegularizer;
    /* Penalty for the rotation in the scan matching process */
    double             mRotationRegularizer;
    /* Cost function */
    CostSquareErrorPtr mCostFunc;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_SCAN_MATCHER_LINEAR_SOLVER_HPP */
