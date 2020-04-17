
/* pose_graph_optimizer.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_POSE_GRAPH_OPTIMIZER_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_POSE_GRAPH_OPTIMIZER_HPP

#include <memory>

#include "my_lidar_graph_slam/mapping/pose_graph.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

class PoseGraphOptimizer
{
public:
    /* Constructor */
    PoseGraphOptimizer() = default;
    /* Destructor */
    virtual ~PoseGraphOptimizer() = default;

    /* Optimize a pose graph */
    virtual void Optimize(std::shared_ptr<PoseGraph>& poseGraph) = 0;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_POSE_GRAPH_OPTIMIZER_HPP */
