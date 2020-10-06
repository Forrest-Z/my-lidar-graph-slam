
/* loop_detector.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_DETECTOR_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_DETECTOR_HPP

#include <memory>
#include <vector>

#include <Eigen/Core>

#include "my_lidar_graph_slam/pose.hpp"
#include "my_lidar_graph_slam/mapping/grid_map_builder.hpp"
#include "my_lidar_graph_slam/mapping/loop_searcher.hpp"
#include "my_lidar_graph_slam/mapping/pose_graph.hpp"
#include "my_lidar_graph_slam/sensor/sensor_data.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/*
 * LoopDetectionQuery struct holds the detailed information for
 * a loop detection, including a collection of nodes and a local grid map.
 * Each node contains its pose in a world frame and an associated scan data
 */
struct LoopDetectionQuery final
{
    /* Constructor */
    LoopDetectionQuery(std::vector<ScanNode>&& scanNodes,
                       const LocalMap& localMap,
                       const LocalMapNode& localMapNode) :
        mScanNodes(std::move(scanNodes)),
        mLocalMap(localMap),
        mLocalMapNode(localMapNode) { }

    /* Destructor */
    ~LoopDetectionQuery() = default;

    /* Vector of scan nodes, each of which is matched against
     * the local grid map using the node pose and its associated scan data */
    const std::vector<ScanNode> mScanNodes;
    /* Local grid map, which is not constant since the precomputation
     * of the coarser grid maps is needed */
    LocalMap                    mLocalMap;
    /* Local map node */
    const LocalMapNode          mLocalMapNode;
};

/* Vector of the loop detection query */
using LoopDetectionQueryVector = std::vector<LoopDetectionQuery>;

/*
 * LoopDetectionResult struct holds the result for a loop detection,
 * necessary information for constructing a pose graph edge that represents
 * a loop closing constraint
 */
struct LoopDetectionResult final
{
    /* Constructor */
    LoopDetectionResult(const RobotPose2D<double>& relativePose,
                        const RobotPose2D<double>& localMapPose,
                        const LocalMapId localMapNodeId,
                        const NodeId scanNodeId,
                        const Eigen::Matrix3d& estimatedCovMat) :
        mRelativePose(relativePose),
        mLocalMapPose(localMapPose),
        mLocalMapNodeId(localMapNodeId),
        mScanNodeId(scanNodeId),
        mEstimatedCovMat(estimatedCovMat) { }

    /* Destructor */
    ~LoopDetectionResult() = default;

    /* Actual relative pose between two nodes */
    const RobotPose2D<double> mRelativePose;
    /* Pose of the start node (local grid map) in a global frame */
    const RobotPose2D<double> mLocalMapPose;
    /* Id of the local map node */
    const LocalMapId          mLocalMapNodeId;
    /* Id of the scan node */
    const NodeId              mScanNodeId;
    /* Estimated covariance matrix in a map-local coordinate frame */
    const Eigen::Matrix3d     mEstimatedCovMat;
};

/* Vector of the loop detection results */
using LoopDetectionResultVector = std::vector<LoopDetectionResult>;

class LoopDetector
{
public:
    /* Constructor */
    LoopDetector() = default;

    /* Destructor */
    virtual ~LoopDetector() = default;

    /* Find a loop and return a loop constraint */
    virtual void Detect(
        LoopDetectionQueryVector& loopDetectionQueries,
        LoopDetectionResultVector& loopDetectionResults) = 0;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_DETECTOR_HPP */
