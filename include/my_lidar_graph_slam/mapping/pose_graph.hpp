
/* pose_graph.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_POSE_GRAPH_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_POSE_GRAPH_HPP

#include "my_lidar_graph_slam/pose.hpp"
#include "my_lidar_graph_slam/mapping/pose_graph_id.hpp"
#include "my_lidar_graph_slam/mapping/pose_graph_id_map.hpp"
#include "my_lidar_graph_slam/mapping/pose_graph_edge.hpp"
#include "my_lidar_graph_slam/mapping/pose_graph_node.hpp"
#include "my_lidar_graph_slam/sensor/sensor_data.hpp"

#include <map>
#include <memory>
#include <optional>
#include <vector>

#include <Eigen/Core>

namespace MyLidarGraphSlam {
namespace Mapping {

/*
 * PoseGraph class represents pose graphs for Graph-Based SLAM
 */
class PoseGraph
{
public:
    /* Constructor */
    PoseGraph() = default;

    /* Destructor */
    ~PoseGraph() = default;

    /* Get the map of the local map nodes */
    inline IdMap<LocalMapId, LocalMapNode>& LocalMapNodes()
    { return this->mLocalMapNodes; }
    /* Get the map of the local map nodes */
    inline const IdMap<LocalMapId, LocalMapNode>& LocalMapNodes() const
    { return this->mLocalMapNodes; }

    /* Get the map of the scan nodes */
    inline IdMap<NodeId, ScanNode>& ScanNodes()
    { return this->mScanNodes; }
    /* Get the map of the scan nodes */
    inline const IdMap<NodeId, ScanNode>& ScanNodes() const
    { return this->mScanNodes; }

    /* Get the vector of the pose graph edges */
    inline std::vector<PoseGraphEdge>& Edges()
    { return this->mEdges; }
    /* Get the vector of the pose graph edges */
    inline const std::vector<PoseGraphEdge>& Edges() const
    { return this->mEdges; }

private:
    /* Map of the local map nodes */
    IdMap<LocalMapId, LocalMapNode> mLocalMapNodes;
    /* Vector of the scan nodes */
    IdMap<NodeId, ScanNode>         mScanNodes;
    /* Vector of the pose graph edges */
    std::vector<PoseGraphEdge>      mEdges;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_POSE_GRAPH_HPP */
