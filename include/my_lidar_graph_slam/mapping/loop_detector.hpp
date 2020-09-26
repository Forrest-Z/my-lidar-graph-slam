
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
    LoopDetectionQuery(
        std::vector<PoseGraph::Node>&& poseGraphNodes,
        const GridMapBuilder::LocalMapInfo& localMapInfo,
        const PoseGraph::Node& localMapNode) :
        mPoseGraphNodes(std::move(poseGraphNodes)),
        mLocalMapInfo(localMapInfo),
        mLocalMapNode(localMapNode) { }

    /* Destructor */
    ~LoopDetectionQuery() = default;

    /* Vector of pose graph nodes, each of which is matched against
     * the local grid map using the node pose and its associated scan data */
    const std::vector<PoseGraph::Node> mPoseGraphNodes;
    /* Local grid map, which is not constant since the precomputation
     * of the coarser grid maps is needed */
    GridMapBuilder::LocalMapInfo       mLocalMapInfo;
    /* Pose graph node that resides in the local map, which is used to
     * specify the node index and to compute the relative pose of the
     * loop closing edge */
    const PoseGraph::Node              mLocalMapNode;
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
                        const RobotPose2D<double>& startNodePose,
                        const int startNodeIdx,
                        const int endNodeIdx,
                        const Eigen::Matrix3d& estimatedCovMat) :
        mRelativePose(relativePose),
        mStartNodePose(startNodePose),
        mStartNodeIdx(startNodeIdx),
        mEndNodeIdx(endNodeIdx),
        mEstimatedCovMat(estimatedCovMat) { }

    /* Destructor */
    ~LoopDetectionResult() = default;

    /* Actual relative pose between two nodes */
    const RobotPose2D<double> mRelativePose;
    /* Pose of the start node */
    const RobotPose2D<double> mStartNodePose;
    /* Index of the start pose graph node */
    const int                 mStartNodeIdx;
    /* Index of the end pose graph node */
    const int                 mEndNodeIdx;
    /* Estimated covariance matrix (inverse of the information matrix) */
    const Eigen::Matrix3d     mEstimatedCovMat;
};

/* Vector of the loop detection results */
using LoopDetectionResultVector = std::vector<LoopDetectionResult>;

class LoopDetector
{
public:
    /* Type definitions */
    using ScanPtr = Sensor::ScanDataPtr<double>;
    using GridMapType = GridMapBuilder::GridMapType;
    using PrecomputedMapType = GridMapBuilder::PrecomputedMapType;

    /* Constructor */
    LoopDetector() = default;

    /* Destructor */
    virtual ~LoopDetector() = default;

    /* Find a loop and return a loop constraint
     * Current scan data stored in the latest pose graph node is
     * matched against the previous local grid map */
    virtual bool FindLoop(
        LoopDetectionQueryVector& loopDetectionQueries,
        LoopDetectionResultVector& loopDetectionResults) = 0;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_DETECTOR_HPP */
