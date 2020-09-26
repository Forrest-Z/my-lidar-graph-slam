
/* lidar_graph_slam_backend.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_LIDAR_GRAPH_SLAM_BACKEND_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_LIDAR_GRAPH_SLAM_BACKEND_HPP

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include "my_lidar_graph_slam/mapping/lidar_graph_slam.hpp"
#include "my_lidar_graph_slam/mapping/loop_detector.hpp"
#include "my_lidar_graph_slam/mapping/loop_searcher.hpp"
#include "my_lidar_graph_slam/mapping/pose_graph_optimizer.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

class LidarGraphSlamBackend
{
public:
    /* Constructor */
    LidarGraphSlamBackend(
        const std::shared_ptr<PoseGraphOptimizer>& poseGraphOptimizer,
        const std::shared_ptr<LoopClosureCandidate>& loopDetectionSearch,
        const std::shared_ptr<LoopClosure>& loopClosure);
    /* Destructor */
    ~LidarGraphSlamBackend() = default;

    /* Copy constructor (disabled) */
    LidarGraphSlamBackend(const LidarGraphSlamBackend&) = delete;
    /* Copy assignment operator (disabled) */
    LidarGraphSlamBackend& operator=(const LidarGraphSlamBackend&) = delete;
    /* Move constructor */
    LidarGraphSlamBackend(LidarGraphSlamBackend&&) = default;
    /* Move assignment operator */
    LidarGraphSlamBackend& operator=(LidarGraphSlamBackend&&) = default;

    /* Run loop detection and pose graph optimization */
    void Run(LidarGraphSlam* const pParent,
             std::atomic<bool>& stopRequest);

private:
    /* Pose graph optimizer */
    std::shared_ptr<PoseGraphOptimizer>   mPoseGraphOptimizer;
    /* Loop detection candidate search */
    std::shared_ptr<LoopClosureCandidate> mLoopDetectionSearch;
    /* Loop closure constraint builder */
    std::shared_ptr<LoopClosure>          mLoopClosure;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_LIDAR_GRAPH_SLAM_BACKEND_HPP */
