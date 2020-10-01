
/* pose_graph.cpp */

#include "my_lidar_graph_slam/mapping/pose_graph.hpp"
#include "my_lidar_graph_slam/util.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/*
 * ScanNodeMap class implementations
 */

/* Get the minimum scan node Id */
NodeId ScanNodeMap::NodeIdMin() const
{
    /* Make sure that the nodes are not empty */
    Assert(!this->mNodes.empty());
    /* Scan node with the smallest Id is at the first element */
    auto firstNodeIt = this->mNodes.begin();
    /* Return the minimum scan node Id */
    return firstNodeIt->first;
}

/* Get the maximum scan node Id */
NodeId ScanNodeMap::NodeIdMax() const
{
    /* Make sure that the nodes are not empty */
    Assert(!this->mNodes.empty());
    /* Scan node with the largest Id is at the last element */
    auto lastNodeIt = std::prev(this->mNodes.end());
    /* Return the maximum scan node Id */
    return lastNodeIt->first;
}

/* Get the latest scan node with the largest node Id */
ScanNode& ScanNodeMap::LatestNode()
{
    /* Use the const version of the method */
    const auto* pThis = static_cast<const ScanNodeMap*>(this);
    return const_cast<ScanNode&>(pThis->LatestNode());
}

/* Get the latest scan node with the largest node Id */
const ScanNode& ScanNodeMap::LatestNode() const
{
    /* Make sure that the nodes are not empty */
    Assert(!this->mNodes.empty());
    /* Assume that the scan node with the largest Id is the latest one */
    auto latestNodeIt = std::prev(this->mNodes.end());
    /* Return the associated scan node */
    return latestNodeIt->second;
}

/* Append a new scan node */
NodeId ScanNodeMap::Append(
    const LocalMapId localMapId,
    const RobotPose2D<double>& localPose,
    const Sensor::ScanDataPtr<double>& scanData,
    const RobotPose2D<double>& globalPose)
{
    /* Determine an Id of a new scan node */
    const NodeId nodeId { this->NewId() };
    /* Insert a new scan node */
    this->mNodes.emplace(
        std::piecewise_construct,
        std::forward_as_tuple(nodeId),
        std::forward_as_tuple(nodeId, localMapId, localPose,
                              scanData, globalPose));
    /* Return the new scan node Id */
    return nodeId;
}

/* Return an Id for a new scan node */
int ScanNodeMap::NewId() const
{
    return this->mNodes.empty() ? 0 : this->NodeIdMax().mId + 1;
}

/*
 * PoseGraph class implementations
 */

/* Append a new local map node and return a new node Id */
void PoseGraph::AppendLocalMapNode(
    const LocalMapId localMapId,
    const RobotPose2D<double>& globalPose)
{
    /* Insert a new local map node */
    this->mLocalMapNodes.emplace(
        std::piecewise_construct,
        std::forward_as_tuple(localMapId),
        std::forward_as_tuple(localMapId, globalPose));
}

/* Append a new scan node and return a new node Id */
NodeId PoseGraph::AppendScanNode(
    const LocalMapId localMapId,
    const RobotPose2D<double>& localPose,
    const Sensor::ScanDataPtr<double>& scanData,
    const RobotPose2D<double>& globalPose)
{
    /* Insert a new scan node */
    return this->mScanNodes.Append(localMapId, localPose,
                                   scanData, globalPose);
}

/* Append a new pose graph edge (constraint) */
void PoseGraph::AppendEdge(
    const LocalMapId localMapNodeId,
    const NodeId scanNodeId,
    const EdgeType edgeType,
    const ConstraintType constraintType,
    const RobotPose2D<double>& relativePose,
    const Eigen::Matrix3d& informationMat)
{
    /* Insert a new pose graph edge */
    this->mEdges.emplace_back(localMapNodeId, scanNodeId,
                              edgeType, constraintType,
                              relativePose, informationMat);
}

/* Get the latest local map node */
LocalMapNode& PoseGraph::LatestLocalMapNode()
{
    /* Use the const version of the method */
    const auto* pThis = static_cast<const PoseGraph*>(this);
    return const_cast<LocalMapNode&>(pThis->LatestLocalMapNode());
}

/* Get the latest local map node */
const LocalMapNode& PoseGraph::LatestLocalMapNode() const
{
    /* Make sure that the pose graph is not empty */
    Assert(!this->mLocalMapNodes.empty());
    /* Assume that the local map node with the largest local map Id
     * is the latest node */
    auto latestNodeIt = std::prev(this->mLocalMapNodes.end());
    /* Return the associated local map node */
    return latestNodeIt->second;
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
