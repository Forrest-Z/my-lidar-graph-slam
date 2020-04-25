
/* loop_closure.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_CLOSURE_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_CLOSURE_HPP

#include <memory>

#include <Eigen/Core>

#include "my_lidar_graph_slam/pose.hpp"
#include "my_lidar_graph_slam/mapping/grid_map_builder.hpp"
#include "my_lidar_graph_slam/mapping/pose_graph.hpp"
#include "my_lidar_graph_slam/sensor/sensor_data.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

class LoopClosure
{
public:
    /* Type definitions */
    using GridMapBuilderPtr = std::shared_ptr<GridMapBuilder>;
    using PoseGraphPtr = std::shared_ptr<Mapping::PoseGraph>;
    using ScanPtr = Sensor::ScanDataPtr<double>;
    using GridMapType = GridMapBuilder::GridMapType;
    using PrecomputedMapType = GridMapBuilder::PrecomputedMapType;

    /* Constructor */
    LoopClosure() = default;

    /* Destructor */
    virtual ~LoopClosure() = default;

    /* Find a loop and return a loop constraint
     * Current scan data stored in the latest pose graph node is
     * matched against the previous local grid map */
    virtual bool FindLoop(GridMapBuilderPtr& gridMapBuilder,
                          const PoseGraphPtr& poseGraph,
                          RobotPose2D<double>& relPose,
                          int& startNodeIdx,
                          int& endNodeIdx,
                          Eigen::Matrix3d& estimatedCovMat) = 0;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_CLOSURE_HPP */
