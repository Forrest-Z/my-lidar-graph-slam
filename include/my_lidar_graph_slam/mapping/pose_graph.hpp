
/* pose_graph.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_POSE_GRAPH_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_POSE_GRAPH_HPP

#include "my_lidar_graph_slam/pose.hpp"
#include "my_lidar_graph_slam/mapping/pose_graph_id.hpp"
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
 * LocalMapNodeMap class provides several helper methods to manipulate
 * local map nodes
 */
class LocalMapNodeMap final
{
public:
    /* Type definition for convenience */
    using MapType = std::map<LocalMapId, LocalMapNode>;

    /* Constructor */
    LocalMapNodeMap() = default;
    /* Destructor */
    ~LocalMapNodeMap() = default;

    /* Get the minimum local map Id */
    LocalMapId NodeIdMin() const;
    /* Get the maximum local map Id */
    LocalMapId NodeIdMax() const;

    /* Get the internal map of the local map nodes */
    inline MapType& Nodes() { return this->mNodes; }
    /* Get the internal map of the local map nodes */
    inline const MapType& Nodes() const { return this->mNodes; }

    /* Get an iterator to the first local map node */
    inline MapType::iterator begin() noexcept
    { return this->mNodes.begin(); }
    /* Get an iterator to the first local map node */
    inline MapType::const_iterator begin() const noexcept
    { return this->mNodes.begin(); }
    /* Get an iterator to the first local map node */
    inline MapType::const_iterator cbegin() const noexcept
    { return this->mNodes.cbegin(); }

    /* Get an iterator to the end local map node */
    inline MapType::iterator end() noexcept
    { return this->mNodes.end(); }
    /* Get an iterator to the end local map node */
    inline MapType::const_iterator end() const noexcept
    { return this->mNodes.end(); }
    /* Get an iterator to the end local map node */
    inline MapType::const_iterator cend() const noexcept
    { return this->mNodes.cend(); }

    /* Get an iterator to the first local map node of the reversed map */
    inline MapType::reverse_iterator rbegin() noexcept
    { return this->mNodes.rbegin(); }
    /* Get an iterator to the first local map node of the reversed map */
    inline MapType::const_reverse_iterator rbegin() const noexcept
    { return this->mNodes.rbegin(); }
    /* Get an iterator to the first local map node of the reversed map */
    inline MapType::const_reverse_iterator crbegin() const noexcept
    { return this->mNodes.crbegin(); }

    /* Get an iterator to the end local map node of the reversed map */
    inline MapType::reverse_iterator rend() noexcept
    { return this->mNodes.rend(); }
    /* Get an iterator to the end local map node of the reversed map */
    inline MapType::const_reverse_iterator rend() const noexcept
    { return this->mNodes.rend(); }
    /* Get an iterator to the end local map node of the reversed map */
    inline MapType::const_reverse_iterator crend() const noexcept
    { return this->mNodes.crend(); }

    /* Get the iterator pointing to the first local map node whose Id is
     * greater than the specified Id */
    inline MapType::const_iterator UpperBound(const LocalMapId nodeId) const
    { return this->mNodes.upper_bound(nodeId); }
    /* Get the iterator pointing to the first local map node whose Id is
     * greater than or equal to the specified Id */
    inline MapType::const_iterator LowerBound(const LocalMapId nodeId) const
    { return this->mNodes.lower_bound(nodeId); }

    /* Clear the local map nodes */
    inline void Clear() { this->mNodes.clear(); }

    /* Check if the local map nodes are empty */
    inline bool Empty() const
    { return this->mNodes.empty(); }
    /* Get the size of the local map nodes */
    inline std::size_t Size() const
    { return this->mNodes.size(); }

    /* Check if the local map node with the specified Id exists */
    inline bool Contains(const LocalMapId localMapId) const
    { return this->mNodes.find(localMapId) != this->mNodes.end(); }

    /* Get the local map node of the specified Id */
    inline LocalMapNode& At(const LocalMapId localMapId)
    { return this->mNodes.at(localMapId); }
    /* Get the local map node of the specified Id */
    inline const LocalMapNode& At(const LocalMapId localMapId) const
    { return this->mNodes.at(localMapId); }

    /* Get the latest local map node with the largest node Id */
    LocalMapNode& LatestNode();
    /* Get the latest local map node with the largest node Id */
    const LocalMapNode& LatestNode() const;

    /* Append a new local map node */
    void Append(const LocalMapId localMapId,
                const RobotPose2D<double>& globalPose);

private:
    /* Map of the local map nodes */
    MapType mNodes;
};

/*
 * ScanNodeMap class provides several helper methods to manipulate
 * scan nodes
 */
class ScanNodeMap final
{
public:
    /* Type definition for convenience */
    using MapType = std::map<NodeId, ScanNode>;

    /* Constructor */
    ScanNodeMap() = default;
    /* Destructor */
    ~ScanNodeMap() = default;

    /* Get the minimum scan node Id */
    NodeId NodeIdMin() const;
    /* Get the maximum scan node Id */
    NodeId NodeIdMax() const;

    /* Get the internal map of the scan nodes */
    inline MapType& Nodes() { return this->mNodes; }
    /* Get the internal map of the scan nodes */
    inline const MapType& Nodes() const { return this->mNodes; }

    /* Get an iterator to the first scan node */
    inline MapType::iterator begin() noexcept
    { return this->mNodes.begin(); }
    /* Get an iterator to the first scan node */
    inline MapType::const_iterator begin() const noexcept
    { return this->mNodes.begin(); }
    /* Get an iterator to the first scan node */
    inline MapType::const_iterator cbegin() const noexcept
    { return this->mNodes.cbegin(); }

    /* Get an iterator to the end scan node */
    inline MapType::iterator end() noexcept
    { return this->mNodes.end(); }
    /* Get an iterator to the end scan node */
    inline MapType::const_iterator end() const noexcept
    { return this->mNodes.end(); }
    /* Get an iterator to the end scan node */
    inline MapType::const_iterator cend() const noexcept
    { return this->mNodes.cend(); }

    /* Get an iterator to the first scan node of the reversed map */
    inline MapType::reverse_iterator rbegin() noexcept
    { return this->mNodes.rbegin(); }
    /* Get an iterator to the first scan node of the reversed map */
    inline MapType::const_reverse_iterator rbegin() const noexcept
    { return this->mNodes.rbegin(); }
    /* Get an iterator to the first scan node of the reversed map */
    inline MapType::const_reverse_iterator crbegin() const noexcept
    { return this->mNodes.crbegin(); }

    /* Get an iterator to the end scan node of the reversed map */
    inline MapType::reverse_iterator rend() noexcept
    { return this->mNodes.rend(); }
    /* Get an iterator to the end scan node of the reversed map */
    inline MapType::const_reverse_iterator rend() const noexcept
    { return this->mNodes.rend(); }
    /* Get an iterator to the end scan node of the reversed map */
    inline MapType::const_reverse_iterator crend() const noexcept
    { return this->mNodes.crend(); }

    /* Get the iterator pointing to the first scan node whose Id is
     * greater than the specified Id */
    inline MapType::const_iterator UpperBound(const NodeId nodeId) const
    { return this->mNodes.upper_bound(nodeId); }
    /* Get the iterator pointing to the first scan node whose Id is
     * greater than or equal to the specified Id */
    inline MapType::const_iterator LowerBound(const NodeId nodeId) const
    { return this->mNodes.lower_bound(nodeId); }

    /* Clear the local map nodes */
    inline void Clear() { this->mNodes.clear(); }

    /* Check if the scan nodes are empty */
    inline bool Empty() const
    { return this->mNodes.empty(); }
    /* Get the size of the scan nodes */
    inline std::size_t Size() const
    { return this->mNodes.size(); }

    /* Check if the scan node with the specified Id exists */
    inline bool Contains(const NodeId nodeId) const
    { return this->mNodes.find(nodeId) != this->mNodes.end(); }

    /* Get the scan node of the specified Id */
    inline ScanNode& At(const NodeId nodeId)
    { return this->mNodes.at(nodeId); }
    /* Get the scan node of the specified Id */
    inline const ScanNode& At(const NodeId nodeId) const
    { return this->mNodes.at(nodeId); }

    /* Get the latest scan node with the largest node Id */
    ScanNode& LatestNode();
    /* Get the latest scan node with the largest node Id */
    const ScanNode& LatestNode() const;

    /* Append a new scan node */
    void Append(const NodeId nodeId,
                const LocalMapId localMapId,
                const RobotPose2D<double>& localPose,
                const Sensor::ScanDataPtr<double>& scanData,
                const RobotPose2D<double>& globalPose);

private:
    /* Map of the scan nodes */
    MapType mNodes;
};

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

    /* Append a new local map node */
    void AppendLocalMapNode(const LocalMapId localMapId,
                            const RobotPose2D<double>& globalPose);

    /* Append a new scan node and return a new node Id */
    void AppendScanNode(const NodeId nodeId,
                        const LocalMapId localMapId,
                        const RobotPose2D<double>& localPose,
                        const Sensor::ScanDataPtr<double>& scanData,
                        const RobotPose2D<double>& globalPose);

    /* Append a new pose graph edge (constraint) */
    void AppendEdge(const LocalMapId localMapNodeId,
                    const NodeId scanNodeId,
                    const EdgeType edgeType,
                    const ConstraintType constraintType,
                    const RobotPose2D<double>& relativePose,
                    const Eigen::Matrix3d& informationMat);

    /* Get the map of the local map nodes */
    inline LocalMapNodeMap& LocalMapNodes()
    { return this->mLocalMapNodes; }
    /* Get the map of the local map nodes */
    inline const LocalMapNodeMap& LocalMapNodes() const
    { return this->mLocalMapNodes; }

    /* Get the local map node of the specified Id */
    inline LocalMapNode& LocalMapNodeAt(const LocalMapId nodeId)
    { return this->mLocalMapNodes.At(nodeId); }
    /* Get the local map node of the specified Id */
    inline const LocalMapNode& LocalMapNodeAt(const LocalMapId nodeId) const
    { return this->mLocalMapNodes.At(nodeId); }

    /* Get the latest local map node */
    inline LocalMapNode& LatestLocalMapNode()
    { return this->mLocalMapNodes.LatestNode(); }
    /* Get the latest local map node */
    inline const LocalMapNode& LatestLocalMapNode() const
    { return this->mLocalMapNodes.LatestNode(); }

    /* Get the map of the scan nodes */
    inline ScanNodeMap& ScanNodes()
    { return this->mScanNodes; }
    /* Get the map of the scan nodes */
    inline const ScanNodeMap& ScanNodes() const
    { return this->mScanNodes; }

    /* Check if the pose graph contains a scan node with the specified Id */
    inline bool ContainsScanNode(const NodeId nodeId) const
    { return this->mScanNodes.Contains(nodeId); }

    /* Get the scan node of the specified Id */
    inline ScanNode& ScanNodeAt(const NodeId nodeId)
    { return this->mScanNodes.At(nodeId); }
    /* Get the scan node of the specified Id */
    inline const ScanNode& ScanNodeAt(const NodeId nodeId) const
    { return this->mScanNodes.At(nodeId); }

    /* Get the latest scan node */
    inline ScanNode& LatestScanNode()
    { return this->mScanNodes.LatestNode(); }
    /* Get the latest scan node */
    inline const ScanNode& LatestScanNode() const
    { return this->mScanNodes.LatestNode(); }

    /* Get the vector of the pose graph edges */
    inline std::vector<PoseGraphEdge>& Edges()
    { return this->mEdges; }
    /* Get the vector of the pose graph edges */
    inline const std::vector<PoseGraphEdge>& Edges() const
    { return this->mEdges; }

    /* Get the pose graph edge of the specified index */
    inline PoseGraphEdge& EdgeAt(const std::size_t edgeIdx)
    { return this->mEdges.at(edgeIdx); }
    /* Get the pose graph edge of the specified index */
    inline const PoseGraphEdge& EdgeAt(const std::size_t edgeIdx) const
    { return this->mEdges.at(edgeIdx); }

private:
    /* Map of the local map nodes */
    LocalMapNodeMap            mLocalMapNodes;
    /* Vector of the scan nodes */
    ScanNodeMap                mScanNodes;
    /* Vector of the pose graph edges */
    std::vector<PoseGraphEdge> mEdges;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_POSE_GRAPH_HPP */
