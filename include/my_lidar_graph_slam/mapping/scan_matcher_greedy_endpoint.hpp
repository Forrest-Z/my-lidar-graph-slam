
/* scan_matcher_greedy_endpoint.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_SCAN_MATCHER_GREEDY_ENDPOINT_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_SCAN_MATCHER_GREEDY_ENDPOINT_HPP

#include "my_lidar_graph_slam/mapping/scan_matcher.hpp"

#include "my_lidar_graph_slam/mapping/cost_function.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

class ScanMatcherGreedyEndpoint final : public ScanMatcher
{
public:
    /* Constructor */
    ScanMatcherGreedyEndpoint(double linearStep,
                              double angularStep,
                              int maxIterations,
                              int maxNumOfRefinements,
                              const CostFuncPtr& costFunc);
    
    /* Destructor */
    ~ScanMatcherGreedyEndpoint() = default;

    /* Optimize the robot pose by scan matching */
    void OptimizePose(const GridMapBase<double>& gridMap,
                      const Sensor::ScanDataPtr<double>& scanData,
                      const RobotPose2D<double>& initialPose,
                      RobotPose2D<double>& estimatedPose,
                      double& normalizedCostValue) override;
    
    /* Calculate a covariance matrix */
    void ComputeCovariance(const GridMapBase<double>& gridMap,
                           const Sensor::ScanDataPtr<double>& scanData,
                           const RobotPose2D<double>& robotPose,
                           Eigen::Matrix3d& estimatedCovMat) override;
    
private:
    /* Initial step of the linear components (x and y) */
    double      mLinearStep;
    /* Initial step of the angular component (theta) */
    double      mAngularStep;
    /* Maximum number of iterations */
    int         mMaxIterations;
    /* Maximum number of step parameter updates */
    int         mMaxNumOfRefinements;
    /* Cost function */
    CostFuncPtr mCostFunc;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_SCAN_MATCHER_GREEDY_ENDPOINT_HPP */
