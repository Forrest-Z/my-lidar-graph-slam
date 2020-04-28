
/* loop_closure_empty.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_CLOSURE_EMPTY_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_CLOSURE_EMPTY_HPP

#include "my_lidar_graph_slam/mapping/loop_closure.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

class LoopClosureEmpty final : public LoopClosure
{
public:
    /* Type definitions */
    using LoopClosure::GridMapBuilderPtr;
    using LoopClosure::PoseGraphPtr;

    /* Constructor */
    LoopClosureEmpty() = default;
    /* Destructor */
    ~LoopClosureEmpty() = default;

    /* Do nothing for a loop closure */
    bool FindLoop(GridMapBuilderPtr& gridMapBuilder,
                  const PoseGraphPtr& poseGraph,
                  RobotPose2D<double>& relPose,
                  int& startNodeIdx,
                  int& endNodeIdx,
                  Eigen::Matrix3d& estimatedCovMat) override;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_CLOSURE_EMPTY_HPP */
