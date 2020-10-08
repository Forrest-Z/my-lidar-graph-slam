
/* pose_graph_node.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_POSE_GRAPH_NODE_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_POSE_GRAPH_NODE_HPP

#include "my_lidar_graph_slam/point.hpp"
#include "my_lidar_graph_slam/pose.hpp"
#include "my_lidar_graph_slam/mapping/pose_graph_id.hpp"
#include "my_lidar_graph_slam/sensor/sensor_data.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

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
 * ScanNodeData struct represents the global pose and Id of the scan node,
 * which is intended for the use in rendering the current pose graph or
 * searching the loop closure candidate, where the corresponding scan data
 * of the scan node is not needed
 */
struct ScanNodeData final
{
    /* Constructor */
    ScanNodeData(const NodeId nodeId,
                 const RobotPose2D<double>& globalPose) :
        mNodeId(nodeId),
        mGlobalPose(globalPose) { }

    /* Destructor */
    ~ScanNodeData() = default;

    /* Comparison (less than) operator */
    inline bool operator<(const ScanNodeData& other) const
    { return this->mNodeId < other.mNodeId; }

    /* Node Id */
    const NodeId              mNodeId;
    /* Robot pose in a world coordinate */
    const RobotPose2D<double> mGlobalPose;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_POSE_GRAPH_NODE_HPP */
