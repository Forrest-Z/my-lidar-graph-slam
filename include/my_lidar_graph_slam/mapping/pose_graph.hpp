
/* pose_graph.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_POSE_GRAPH_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_POSE_GRAPH_HPP

#include "my_lidar_graph_slam/pose.hpp"
#include "my_lidar_graph_slam/sensor/sensor_data.hpp"

#include <map>
#include <memory>
#include <optional>
#include <vector>

#include <Eigen/Core>

namespace MyLidarGraphSlam {
namespace Mapping {

/*
 * LocalMapId struct represents an Id value for a local grid map
 */
struct LocalMapId final
{
    /* Constructor */
    explicit LocalMapId(const int localMapId) : mId(localMapId) { }

    /* Equality operator */
    inline bool operator==(const LocalMapId& other) const
    { return this->mId == other.mId; }
    /* Inequality operator */
    inline bool operator!=(const LocalMapId& other) const
    { return !operator==(other); }

    /* Less-than comparison operator (for std::map) */
    inline bool operator<(const LocalMapId& other) const
    { return this->mId < other.mId; }
    /* Greater-than comparison operator */
    inline bool operator>(const LocalMapId& other) const
    { return this->mId > other.mId; }
    /* Less-than or equal to comparison operator */
    inline bool operator<=(const LocalMapId& other) const
    { return this->mId <= other.mId; }
    /* Greater-than or equal to comparison operator */
    inline bool operator>=(const LocalMapId& other) const
    { return this->mId >= other.mId; }

    /* Local grid map Id */
    int mId;
};

/*
 * LocalMapNode struct represents a pose graph node for a local grid map
 */
struct LocalMapNode final
{
    /* Constructor */
    LocalMapNode(const LocalMapId localMapId,
                 const RobotPose2D<double>& globalPose) :
        mLocalMapId(localMapId),
        mGlobalPose(globalPose) { }

    /* Destructor */
    ~LocalMapNode() = default;

    /* Local grid map Id */
    const LocalMapId    mLocalMapId;
    /* Node pose in a world frame */
    RobotPose2D<double> mGlobalPose;
};

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

    /* Get the map of the local map nodes */
    inline const MapType& Nodes() const
    { return this->mNodes; }

    /* Get the iterator pointing to the first local map node whose Id is
     * greater than the specified Id */
    inline MapType::const_iterator UpperBound(const LocalMapId nodeId) const
    { return this->mNodes.upper_bound(nodeId); }
    /* Get the iterator pointing to the first local map node whose Id is
     * greater than or equal to the specified Id */
    inline MapType::const_iterator LowerBound(const LocalMapId nodeId) const
    { return this->mNodes.lower_bound(nodeId); }

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
 * PoseGraphNodeId struct represents an Id value for a pose graph node
 */
struct NodeId final
{
    /* Constructor */
    explicit NodeId(const int nodeId) : mId(nodeId) { }

    /* Equality operator */
    inline bool operator==(const NodeId& other) const
    { return this->mId == other.mId; }
    /* Inequality operator */
    inline bool operator!=(const NodeId& other) const
    { return !operator==(other); }

    /* Less-than comparison operator (for std::map) */
    inline bool operator<(const NodeId& other) const
    { return this->mId < other.mId; }
    /* Greater-than comparison operator */
    inline bool operator>(const NodeId& other) const
    { return this->mId > other.mId; }
    /* Less-than or equal to comparison operator */
    inline bool operator<=(const NodeId& other) const
    { return this->mId <= other.mId; }
    /* Greater-than or equal to comparison operator */
    inline bool operator>=(const NodeId& other) const
    { return this->mId >= other.mId; }

    /* Pose graph node Id */
    int mId;
};

/*
 * ScanNode struct represents a pose graph node for a scan data
 * ScanNode belongs to a local grid map, inside of which the scan data
 * is acquired, and has a pose in a coordinate frame centered at the
 * origin of this local grid map
 */
struct ScanNode final
{
    /* Constructor */
    ScanNode(const NodeId nodeId,
             const LocalMapId localMapId,
             const RobotPose2D<double>& localPose,
             const Sensor::ScanDataPtr<double>& scanData,
             const RobotPose2D<double>& globalPose) :
        mNodeId(nodeId),
        mLocalMapId(localMapId),
        mLocalPose(localPose),
        mScanData(scanData),
        mGlobalPose(globalPose) { }

    /* Destructor */
    ~ScanNode() = default;

    /* Node Id */
    const NodeId                      mNodeId;
    /* Local grid map Id */
    const LocalMapId                  mLocalMapId;
    /* Node pose in a local frame */
    const RobotPose2D<double>         mLocalPose;
    /* Scan data */
    const Sensor::ScanDataPtr<double> mScanData;
    /* Node pose in a world frame */
    RobotPose2D<double>               mGlobalPose;
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

    /* Get the map of the scan nodes */
    inline const MapType& Nodes() const
    { return this->mNodes; }

    /* Get the iterator pointing to the first scan node whose Id is
     * greater than the specified Id */
    inline MapType::const_iterator UpperBound(const NodeId nodeId) const
    { return this->mNodes.upper_bound(nodeId); }
    /* Get the iterator pointing to the first scan node whose Id is
     * greater than or equal to the specified Id */
    inline MapType::const_iterator LowerBound(const NodeId nodeId) const
    { return this->mNodes.lower_bound(nodeId); }

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
    NodeId Append(const LocalMapId localMapId,
                  const RobotPose2D<double>& localPose,
                  const Sensor::ScanDataPtr<double>& scanData,
                  const RobotPose2D<double>& globalPose);

private:
    /* Return an Id for a new scan node */
    int NewId() const;

private:
    /* Map of the scan nodes */
    MapType mNodes;
};

/*
 * EdgeType enum represents whether a pose graph edge is an intra-local
 * grid map constraint or an inter-local grid map constraint
 */
enum class EdgeType
{
    IntraLocalMap,
    InterLocalMap,
};

/*
 * ConstraintType enum represents whether a pose graph edge is an odometry
 * constraint or a loop closing constraint
 */
enum class ConstraintType
{
    Odometry,
    Loop,
};

/*
 * PoseGraphEdge class represents pose graph edges (constraints)
 * connecting a local grid map node and a scan data node
 */
struct PoseGraphEdge final
{
    /* Constructor */
    PoseGraphEdge(const LocalMapId localMapNodeId,
                  const NodeId scanNodeId,
                  const EdgeType edgeType,
                  const ConstraintType constraintType,
                  const RobotPose2D<double>& relativePose,
                  const Eigen::Matrix3d& informationMat) :
        mLocalMapNodeId(localMapNodeId),
        mScanNodeId(scanNodeId),
        mEdgeType(edgeType),
        mConstraintType(constraintType),
        mRelativePose(relativePose),
        mInformationMat(informationMat) { }

    /* Destructor */
    ~PoseGraphEdge() = default;

    /* Return if this edge represents an intra-local grid map constraint,
     * that is, the scan data with an Id `mScanNodeId` is acquired at its
     * associated local grid map with an Id `mLocalMapNodeId` */
    inline bool IsIntraLocalMap() const
    { return this->mEdgeType == EdgeType::IntraLocalMap; }

    /* Return if this edge represents an inter-local grid map constraint,
     * that is, the scan data with an Id `mScanNodeId` is not acquired at
     * its associated local grid map with an Id `mLocalMapNodeId` and
     * the scan data `mScanNodeId` does not belong to the
     * local grid map `mLocalMapNodeId` */
    inline bool IsInterLocalMap() const
    { return this->mEdgeType == EdgeType::InterLocalMap; }

    /* Return if this edge represents an odometry constraint */
    inline bool IsOdometryConstraint() const
    { return this->mConstraintType == ConstraintType::Odometry; }

    /* Return if this edge represents a loop closing constraint */
    inline bool IsLoopClosingConstraint() const
    { return this->mConstraintType == ConstraintType::Loop; }

    /* Local grid map node Id */
    const LocalMapId          mLocalMapNodeId;
    /* Scan data node Id */
    const NodeId              mScanNodeId;
    /* Enumerator to represent whether this edge represents an intra-local
     * grid map constraint or an inter-local grid map constraint */
    const EdgeType            mEdgeType;
    /* Enumerator to represent whether this edge represents an odometry
     * constraint or a loop closing constraint */
    const ConstraintType      mConstraintType;
    /* Relative pose between two pose graph nodes */
    const RobotPose2D<double> mRelativePose;
    /* Information matrix (inverse of the covariance matrix) */
    const Eigen::Matrix3d     mInformationMat;
};

/*
 * NodePosition struct represents the pose and index of
 * the pose graph nodes, which is intended for the use in
 * rendering the current pose graph or searching the loop closure
 * candidate, where the corresponding scan data of the node is not needed
 */
struct NodePosition final
{
    /* Constructor */
    NodePosition(int nodeIdx,
                 const RobotPose2D<double>& pose) :
        mIdx(nodeIdx), mPose(pose) { }
    /* Destructor */
    ~NodePosition() = default;

    /* Comparison (less than) operator */
    inline bool operator<(const NodePosition& other) const
    { return this->mIdx < other.mIdx; }

    /* Index of the node */
    const int                 mIdx;
    /* Robot pose */
    const RobotPose2D<double> mPose;
};

/*
 * EdgeConnection struct represents the indices of the adjacent
 * pose graph nodes and the type (odometry or loop constraint), which is
 * intended for the use in rendering the current pose graph, where
 * the corresponding information matrix is not needed
 */
struct EdgeConnection final
{
    /* Constructor */
    EdgeConnection(int startNodeIdx,
                   int endNodeIdx,
                   bool isOdometry) :
        mStartNodeIdx(startNodeIdx),
        mEndNodeIdx(endNodeIdx),
        mIsOdometry(isOdometry) { }
    /* Destructor */
    ~EdgeConnection() = default;

    /* Index of the start node */
    const int  mStartNodeIdx;
    /* Index of the end node */
    const int  mEndNodeIdx;
    /* Flag to determine whether the edge represents
     * odometry or loop constraint */
    const bool mIsOdometry;
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
    NodeId AppendScanNode(const LocalMapId localMapId,
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
