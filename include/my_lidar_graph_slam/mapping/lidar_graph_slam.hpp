
/* lidar_graph_slam.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_LIDAR_GRAPH_SLAM_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_LIDAR_GRAPH_SLAM_HPP

#include <atomic>
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

#include "my_lidar_graph_slam/pose.hpp"
#include "my_lidar_graph_slam/grid_map/binary_bayes_grid_cell.hpp"
#include "my_lidar_graph_slam/mapping/grid_map_builder.hpp"
#include "my_lidar_graph_slam/mapping/loop_detector.hpp"
#include "my_lidar_graph_slam/mapping/loop_searcher.hpp"
#include "my_lidar_graph_slam/mapping/pose_graph.hpp"
#include "my_lidar_graph_slam/mapping/pose_graph_optimizer.hpp"
#include "my_lidar_graph_slam/mapping/scan_interpolator.hpp"
#include "my_lidar_graph_slam/mapping/scan_matcher.hpp"
#include "my_lidar_graph_slam/sensor/sensor_data.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/* Type definitions for convenience */
class LidarGraphSlam;
using LidarGraphSlamPtr = std::shared_ptr<LidarGraphSlam>;
using LidarGraphSlamWeakPtr = std::weak_ptr<LidarGraphSlam>;

/* Forward declarations */
class LidarGraphSlamBackend;
class LidarGraphSlamFrontend;

class LidarGraphSlam
{
public:
    /* Type definitions */
    using FrontendType = LidarGraphSlamFrontend;
    using BackendType = LidarGraphSlamBackend;

    /* Constructor */
    LidarGraphSlam(
        const std::shared_ptr<FrontendType>& slamFrontend,
        const std::shared_ptr<BackendType>& slamBackend,
        const std::shared_ptr<GridMapBuilder>& gridMapBuilder,
        const std::shared_ptr<PoseGraph>& poseGraph);

    /* Destructor */
    ~LidarGraphSlam() = default;

    /* Copy constructor (disabled) */
    LidarGraphSlam(const LidarGraphSlam&) = delete;
    /* Copy assignment operator (disabled) */
    LidarGraphSlam& operator=(const LidarGraphSlam&) = delete;
    /* Move constructor (disabled) */
    LidarGraphSlam(LidarGraphSlam&&) = delete;
    /* Move assignment operator (disabled) */
    LidarGraphSlam& operator=(LidarGraphSlam&&) = delete;

    /* Process scan data and odometry */
    bool ProcessScan(const Sensor::ScanDataPtr<double>& rawScanData,
                     const RobotPose2D<double>& odomPose);

    /* Retrieve the total number of the processed input data */
    int ProcessCount() const;

    /* Retrieve the full pose graph information */
    void GetPoseGraph(
        LocalMapNodeMap& localMapNodes,
        ScanNodeMap& scanNodes,
        std::vector<PoseGraphEdge>& poseGraphEdges) const;
    /* Retrieve the pose graph information */
    void GetPoseGraph(
        std::map<LocalMapId, LocalMapData>& localMapNodes,
        std::map<NodeId, ScanNodeData>& scanNodes,
        std::vector<EdgeData>& poseGraphEdges) const;

    /* Retrieve the latest data consisting of the pose from the last scan nodes
     * (last robot pose in a global coordinate frame), the latest map obtained
     * from the several recently acquired scans, and the pose of the latest map
     * in a global coordinate frame */
    void GetLatestData(
        RobotPose2D<double>& lastScanPose,
        GridMapType& latestMap,
        RobotPose2D<double>& latestMapPose) const;

    /* Retrieve the necessary information for loop search */
    LoopSearchHint GetLoopSearchHint() const;
    /* Retrieve the necessary information for loop detection */
    LoopDetectionQueryVector GetLoopDetectionQueries(
        const LoopCandidateVector& loopCandidates) const;

    /* Append a first node with an associated scan data and update the
     * current local grid map and the latest map */
    void AppendFirstNodeAndEdge(
        const RobotPose2D<double>& initialScanPose,
        const Sensor::ScanDataPtr<double>& scanData);

    /* Append a new pose graph node and an odometry edge, and update the
     * current local grid map and the latest map */
    void AppendNodeAndEdge(
        const RobotPose2D<double>& relativeScanPose,
        const Eigen::Matrix3d& edgeCovarianceMatrix,
        const Sensor::ScanDataPtr<double>& scanData);

    /* Append new loop closing edges */
    void AppendLoopClosingEdges(
        const LoopDetectionResultVector& loopDetectionResults);

    /* Update the coarser grid maps if modified in the loop detection process */
    void UpdatePrecomputedGridMaps(
        LoopDetectionQueryVector& loopDetectionQueries);

    /* Rebuild grid maps after loop closure */
    void AfterLoopClosure(
        const LocalMapNodeMap& localMapNodes,
        const ScanNodeMap& scanNodes);

    /* Retrieve a latest map that contains latest scans */
    void GetLatestMap(RobotPose2D<double>& globalPose,
                      GridMapType& latestMap,
                      NodeId& scanNodeIdMin,
                      NodeId& scanNodeIdMax) const;
    /* Build a global map that contains all local grid maps acquired */
    void GetGlobalMap(RobotPose2D<double>& globalPose,
                      GridMapType& globalMap,
                      NodeId& scanNodeIdMin,
                      NodeId& scanNodeIdMax) const;

    /* Start the SLAM backend */
    void StartBackend();
    /* Stop the SLAM backend */
    void StopBackend();
    /* Notify the SLAM backend */
    void NotifyBackend();
    /* Wait for the notification from the SLAM frontend */
    void WaitForNotification();

private:
    /* SLAM frontend (scan matching and pose graph construction) */
    std::shared_ptr<FrontendType>   mFrontend;
    /* SLAM backend (loop detection and pose graph optimization) */
    std::shared_ptr<BackendType>    mBackend;
    /* Worker thread that runs SLAM backend */
    std::shared_ptr<std::thread>    mBackendThread;
    /* Flag to stop the SLAM backend */
    std::atomic<bool>               mBackendStopRequest;
    /* Condition variable for notification to SLAM backend */
    std::condition_variable         mBackendNotifyCond;
    /* Flag for notification to the SLAM backend */
    bool                            mBackendNotify;
    /* Grid map */
    std::shared_ptr<GridMapBuilder> mGridMapBuilder;
    /* Pose graph */
    std::shared_ptr<PoseGraph>      mPoseGraph;
    /* Shared mutex */
    mutable std::mutex              mMutex;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_LIDAR_GRAPH_SLAM_HPP */
