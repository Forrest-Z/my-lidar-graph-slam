
/* pose_graph.cpp */

#include "my_lidar_graph_slam/mapping/pose_graph.hpp"
#include "my_lidar_graph_slam/util.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/*
 * LocalMapNodeMap class implementations
 */

/* Get the minimum local map Id */
LocalMapId LocalMapNodeMap::NodeIdMin() const
{
    /* Make sure that the local map nodes are not empty */
    Assert(!this->mNodes.empty());
    /* Local map node with the smallest Id is at the first element */
    auto firstNodeIt = this->mNodes.begin();
    /* Return the minimum local map Id */
    return firstNodeIt->first;
}

/* Get the maximum local map Id */
LocalMapId LocalMapNodeMap::NodeIdMax() const
{
    /* Make sure that the local map nodes are not empty */
    Assert(!this->mNodes.empty());
    /* Local map node with the largest Id is at the last element */
    auto lastNodeIt = std::prev(this->mNodes.end());
    /* Return the maximum local map Id */
    return lastNodeIt->first;
}

/* Get the latest local map node with the largest node Id */
LocalMapNode& LocalMapNodeMap::LatestNode()
{
    /* Use the const version of the method */
    const auto* pThis = static_cast<const LocalMapNodeMap*>(this);
    return const_cast<LocalMapNode&>(pThis->LatestNode());
}

/* Get the latest local map node with the largest node Id */
const LocalMapNode& LocalMapNodeMap::LatestNode() const
{
    /* Make sure that the local map nodes are not empty */
    Assert(!this->mNodes.empty());
    /* Assume that the local map node with the largest Id is the latest one */
    auto latestNodeIt = std::prev(this->mNodes.end());
    /* Return the associated local map node */
    return latestNodeIt->second;
}

/* Append a new local map node */
void LocalMapNodeMap::Append(
    const LocalMapId localMapId,
    const RobotPose2D<double>& globalPose)
{
    /* Insert a new local map node */
    this->mNodes.emplace(
        std::piecewise_construct,
        std::forward_as_tuple(localMapId),
        std::forward_as_tuple(localMapId, globalPose));
}

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
    this->mLocalMapNodes.Append(localMapId, globalPose);
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

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
