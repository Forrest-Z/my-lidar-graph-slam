
/* loop_closure_empty.cpp */

#include "my_lidar_graph_slam/mapping/loop_closure_empty.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/* Find a loop and return a loop constraint */
bool LoopClosureEmpty::FindLoop(
    std::shared_ptr<GridMapBuilder>& gridMapBuilder,
    const std::shared_ptr<PoseGraph>& poseGraph,
    RobotPose2D<double>& relPose,
    int& startNodeIdx,
    int& endNodeIdx,
    Eigen::Matrix3d& estimatedCovMat)
{
    /* Do not perform loop closure */
    relPose = RobotPose2D<double>(0.0, 0.0, 0.0);
    startNodeIdx = 0;
    endNodeIdx = 0;
    estimatedCovMat = Eigen::Matrix3d::Zero();

    return false;
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
