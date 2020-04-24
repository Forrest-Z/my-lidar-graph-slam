
/* loop_closure_candidate.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_CLOSURE_CANDIDATE_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_CLOSURE_CANDIDATE_HPP

#include <memory>
#include <utility>
#include <vector>

#include "my_lidar_graph_slam/pose.hpp"
#include "my_lidar_graph_slam/mapping/grid_map_builder.hpp"
#include "my_lidar_graph_slam/mapping/pose_graph.hpp"
#include "my_lidar_graph_slam/sensor/sensor_data.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

class LoopClosureCandidate
{
public:
    /* Type definitions */
    using GridMapBuilderPtr = std::shared_ptr<GridMapBuilder>;
    using PoseGraphPtr = std::shared_ptr<Mapping::PoseGraph>;
    using ScanPtr = Sensor::ScanDataPtr<double>;
    using GridMapType = GridMapBuilder::GridMapType;

    /* CandidateType holds a local map index and a pose graph node index */
    using CandidateType = std::pair<int, int>;
    using CandidateVector = std::vector<CandidateType>;

    /* Constructor */
    LoopClosureCandidate() = default;

    /* Destructor */
    virtual ~LoopClosureCandidate() = default;

    /* Find a local map and a pose graph node used for loop closure */
    virtual CandidateVector Find(const GridMapBuilderPtr& gridMapBuilder,
                                 const PoseGraphPtr& poseGraph,
                                 const RobotPose2D<double>& robotPose) = 0;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_CLOSURE_CANDIDATE_HPP */
