
/* pose_graph.cpp */

#include "my_lidar_graph_slam/mapping/pose_graph.hpp"
#include "my_lidar_graph_slam/util.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

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
    /* Determine an Id of a new scan node */
    const NodeId nodeId { static_cast<int>(this->mScanNodes.size()) };
    /* Insert a new scan node */
    this->mScanNodes.emplace(
        std::piecewise_construct,
        std::forward_as_tuple(nodeId),
        std::forward_as_tuple(nodeId, localMapId, localPose,
                              scanData, globalPose));
    /* Return the new scan node Id */
    return nodeId;
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

/* Get the latest scan node */
ScanNode& PoseGraph::LatestScanNode()
{
    /* Use the const version of the method */
    const auto* pThis = static_cast<const PoseGraph*>(this);
    return const_cast<ScanNode&>(pThis->LatestScanNode());
}

/* Get the latest scan node */
const ScanNode& PoseGraph::LatestScanNode() const
{
    /* Make sure that the pose graph is not empty */
    Assert(!this->mScanNodes.empty());
    /* Assume that the scan node with the largest node Id is the latest node */
    auto latestNodeIt = std::prev(this->mScanNodes.end());
    /* Return the associated scan node */
    return latestNodeIt->second;
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
