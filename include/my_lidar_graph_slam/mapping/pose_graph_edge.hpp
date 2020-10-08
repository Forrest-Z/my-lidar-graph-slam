
/* pose_graph_node.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_POSE_GRAPH_EDGE_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_POSE_GRAPH_EDGE_HPP

#include <Eigen/Core>

#include "my_lidar_graph_slam/point.hpp"
#include "my_lidar_graph_slam/pose.hpp"
#include "my_lidar_graph_slam/mapping/pose_graph_id.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

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
 * EdgeData struct represents the Ids of the two adjacent pose graph nodes
 * and the type (odometry or loop constraint, intra-local or inter-local),
 * which is intended for the use in rendering the current pose graph, where
 * the corresponding information matrix and the relative pose are not needed
 */
struct EdgeData final
{
    /* Constructor */
    EdgeData(const LocalMapId localMapNodeId,
             const NodeId scanNodeId,
             const EdgeType edgeType,
             const ConstraintType constraintType) :
        mLocalMapNodeId(localMapNodeId),
        mScanNodeId(scanNodeId),
        mEdgeType(edgeType),
        mConstraintType(constraintType) { }
    /* Destructor */
    ~EdgeData() = default;

    /* Local map node Id */
    const LocalMapId     mLocalMapNodeId;
    /* Scan data node Id */
    const NodeId         mScanNodeId;
    /* Edge type (intra-local or inter-local) */
    const EdgeType       mEdgeType;
    /* Constraint type (odometry or loop) */
    const ConstraintType mConstraintType;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_POSE_GRAPH_EDGE_HPP */
