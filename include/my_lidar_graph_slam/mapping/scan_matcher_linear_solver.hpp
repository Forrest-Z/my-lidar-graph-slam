
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
    ScanMatcherLinearSolver(const int numOfIterationsMax,
                            const double convergenceThreshold,
                            const double usableRangeMin,
                            const double usableRangeMax,
                            const double translationRegularizer,
                            const double rotationRegularizer,
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
        const RobotPose2D<double>& mapLocalSensorPose);

private:
    /* Maximum number of the optimization iterations */
    const int          mNumOfIterationsMax;
    /* Threshold to check the convergence */
    const double       mConvergenceThreshold;
    /* Minimum laser scan range considered for calculation */
    const double       mUsableRangeMin;
    /* Maximum laser scan range considered for calculation */
    const double       mUsableRangeMax;
    /* Penalty for the translation in the scan matching process */
    const double       mTranslationRegularizer;
    /* Penalty for the rotation in the scan matching process */
    const double       mRotationRegularizer;
    /* Cost function */
    CostSquareErrorPtr mCostFunc;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_SCAN_MATCHER_LINEAR_SOLVER_HPP */
