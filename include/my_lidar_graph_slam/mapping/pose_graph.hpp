
/* pose_graph.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_POSE_GRAPH_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_POSE_GRAPH_HPP

#include "my_lidar_graph_slam/pose.hpp"
#include "my_lidar_graph_slam/sensor/sensor_data.hpp"

#include <memory>
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

    /* Local grid map Id */
    const int mId;
};

/*
 * LocalMapNode struct represents a pose graph node for a local grid map
 */
struct LocalMapNode final
{
    /* Constructor */
    LocalMapNode(const int localMapId,
                 const RobotPose2D<double>& globalPose) :
        mLocalMapId(localMapId),
        mGlobalPose(globalPose) { }

    /* Destructor */
    ~LocalMapNode() = default;

    /* Local grid map Id */
    const int           mLocalMapId;
    /* Node pose in a world frame */
    RobotPose2D<double> mGlobalPose;
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

    /* Pose graph node Id */
    const int mId;
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
    ScanNode(const int nodeId,
             const int localMapId,
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
    const int                         mNodeId;
    /* Local grid map Id */
    const int                         mLocalMapId;
    /* Node pose in a local frame */
    const RobotPose2D<double>         mLocalPose;
    /* Scan data */
    const Sensor::ScanDataPtr<double> mScanData;
    /* Node pose in a world frame */
    RobotPose2D<double>               mGlobalPose;
};

/*
 * PoseGraphEdge class represents pose graph edges (constraints)
 * connecting a local grid map node and a scan data node
 */
struct PoseGraphEdge final
{
    /* Constructor */
    PoseGraphEdge(const int localMapNodeId,
                  const int scanNodeId,
                  const bool isIntraLocalMap,
                  const RobotPose2D<double>& relativePose,
                  const Eigen::Matrix3d& informationMat) :
        mLocalMapNodeId(localMapNodeId),
        mScanNodeId(scanNodeId),
        mIsIntraLocalMap(isIntraLocalMap),
        mRelativePose(relativePose),
        mInformationMat(informationMat) { }

    /* Destructor */
    ~PoseGraphEdge() = default;

    /* Return if this edge represents an intra-local grid map constraint,
     * that is, the scan data with an Id `mScanNodeId` is acquired at its
     * associated local grid map with an Id `mLocalMapNodeId` */
    inline bool IsIntraLocalMap() const { return this->mIsIntraLocalMap; }

    /* Return if this edge represents an inter-local grid map constraint,
     * that is, the scan data with an Id `mScanNodeId` is not acquired at
     * its associated local grid map with an Id `mLocalMapNodeId` and
     * the scan data `mScanNodeId` does not belong to the
     * local grid map `mLocalMapNodeId` */
    inline bool IsInterLocalMap() const { return !this->IsIntraLocalMap(); }

    /* Local grid map node Id */
    const int                 mLocalMapNodeId;
    /* Scan data node Id */
    const int                 mScanNodeId;
    /* Flag to represent whether this edge represents an intra-local grid map
     * constraint or an inter-local grid map constraint */
    const bool                mIsIntraLocalMap;
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

    /* Append new node (new node index is returned) */
    int AppendNode(const RobotPose2D<double>& pose,
                   const Sensor::ScanDataPtr<double>& scanData);
    
    /* Append new edge */
    void AppendEdge(int startNodeIdx,
                    int endNodeIdx,
                    const RobotPose2D<double>& relativePose,
                    const Eigen::Matrix3d& informationMat);
    
    /* Retrieve the list of the pose graph nodes */
    inline const std::vector<Node>& Nodes() const { return this->mNodes; }
    
    /* Retrieve the node of the specified index */
    inline Node& NodeAt(int nodeIdx)
    { return this->mNodes.at(nodeIdx); }
    /* Retrieve the node of the specified index */
    inline const Node& NodeAt(int nodeIdx) const
    { return this->mNodes.at(nodeIdx); }
    
    /* Retrieve the latest node */
    inline Node& LatestNode() { return this->mNodes.back(); }
    /* Retrieve the latest node */
    inline const Node& LatestNode() const { return this->mNodes.back(); }

    /* Retrieve the list of the pose graph edges */
    inline const std::vector<Edge>& Edges() const { return this->mEdges; }

    /* Retrieve the edge of the specified index */
    inline Edge& EdgeAt(int edgeIdx)
    { return this->mEdges.at(edgeIdx); }
    /* Retrieve the edge of the specified index */
    inline const Edge& EdgeAt(int edgeIdx) const
    { return this->mEdges.at(edgeIdx); }

private:
    /* List of the pose graph nodes */
    std::vector<Node> mNodes;
    /* List of the pose graph edges */
    std::vector<Edge> mEdges;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_POSE_GRAPH_HPP */
