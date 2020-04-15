
/* pose_graph.cpp */

#include "my_lidar_graph_slam/mapping/pose_graph.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/*
 * PoseGraph class implementations
 */

/* Append new node (new node index is returned) */
int PoseGraph::AppendNode(const RobotPose2D<double>& pose,
                          const Sensor::ScanDataPtr<double>& scanData)
{
    /* Determine the index of the new node */
    const int nodeIdx = static_cast<int>(this->mNodes.size());
    /* Append the new node */
    this->mNodes.emplace_back(nodeIdx, pose, scanData);
    
    return nodeIdx;
}

/* Append new edge */
void PoseGraph::AppendEdge(int startNodeIdx,
                           int endNodeIdx,
                           const RobotPose2D<double>& relativePose,
                           const Eigen::Matrix3d& informationMat)
{
    /* Append the new edge */
    this->mEdges.emplace_back(startNodeIdx, endNodeIdx,
                              relativePose, informationMat);
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
