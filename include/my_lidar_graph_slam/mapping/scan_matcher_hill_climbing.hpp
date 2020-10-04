
/* scan_matcher_hill_climbing.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_SCAN_MATCHER_HILL_CLIMBING_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_SCAN_MATCHER_HILL_CLIMBING_HPP

#include "my_lidar_graph_slam/mapping/scan_matcher.hpp"

#include "my_lidar_graph_slam/mapping/cost_function.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

class ScanMatcherHillClimbing final : public ScanMatcher
{
public:
    /* Constructor */
    ScanMatcherHillClimbing(const double linearStep,
                            const double angularStep,
                            const int maxIterations,
                            const int maxNumOfRefinements,
                            const CostFuncPtr& costFunc);

    /* Destructor */
    ~ScanMatcherHillClimbing() = default;

    /* Optimize the robot pose by scan matching */
    ScanMatchingSummary OptimizePose(
        const ScanMatchingQuery& queryInfo) override;

private:
    /* Initial step of the linear components (x and y) */
    const double      mLinearStep;
    /* Initial step of the angular component (theta) */
    const double      mAngularStep;
    /* Maximum number of iterations */
    const int         mMaxIterations;
    /* Maximum number of step parameter updates */
    const int         mMaxNumOfRefinements;
    /* Cost function */
    const CostFuncPtr mCostFunc;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_SCAN_MATCHER_HILL_CLIMBING_HPP */
