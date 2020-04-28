
/* lidar_graph_slam.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_LIDAR_GRAPH_SLAM_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_LIDAR_GRAPH_SLAM_HPP

#include <memory>
#include <utility>

#include "my_lidar_graph_slam/pose.hpp"
#include "my_lidar_graph_slam/grid_map/binary_bayes_grid_cell.hpp"
#include "my_lidar_graph_slam/mapping/grid_map_builder.hpp"
#include "my_lidar_graph_slam/mapping/loop_closure_grid_search.hpp"
#include "my_lidar_graph_slam/mapping/pose_graph.hpp"
#include "my_lidar_graph_slam/mapping/pose_graph_optimizer.hpp"
#include "my_lidar_graph_slam/mapping/scan_matcher.hpp"
#include "my_lidar_graph_slam/sensor/sensor_data.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

class LidarGraphSlam
{
public:
    /* Type definitions */
    using GridMapType = GridMap<BinaryBayesGridCell<double>>;

    /* Constructor */
    LidarGraphSlam(
        const std::shared_ptr<GridMapBuilder>& gridMapBuilder,
        const ScanMatcherPtr& scanMatcher,
        const std::shared_ptr<PoseGraph>& poseGraph,
        const std::shared_ptr<PoseGraphOptimizer>& poseGraphOptimizer,
        const std::shared_ptr<LoopClosure>& loopClosure,
        int loopClosureInterval,
        const RobotPose2D<double>& initialPose,
        double updateThresholdTravelDist,
        double updateThresholdAngle,
        double updateThresholdTime);

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
    bool ProcessScan(const Sensor::ScanDataPtr<double>& scanData,
                     const RobotPose2D<double>& odomPose);
    
    /* Retrieve the process counter */
    inline int ProcessCount() const { return this->mProcessCount; }

    /* Retrieve the grid map builder object */
    inline const std::shared_ptr<GridMapBuilder>& GetGridMapBuilder() const
    { return this->mGridMapBuilder; }
    
    /* Retrieve the pose graph */
    inline const std::shared_ptr<PoseGraph>& GetPoseGraph() const
    { return this->mPoseGraph; }

private:
    /* Process counter (the total number of input data) */
    int                                 mProcessCount;
    /* Grid map */
    std::shared_ptr<GridMapBuilder>     mGridMapBuilder;
    /* Scan matcher */
    ScanMatcherPtr                      mScanMatcher;
    /* Pose graph */
    std::shared_ptr<PoseGraph>          mPoseGraph;
    /* Pose graph optimizer */
    std::shared_ptr<PoseGraphOptimizer> mPoseGraphOptimizer;
    /* Loop closure */
    std::shared_ptr<LoopClosure>        mLoopClosure;
    /* Frame interval for loop closure */
    int                                 mLoopClosureInterval;
    /* Initial pose */
    RobotPose2D<double>                 mInitialPose;
    /* Last odometry pose */
    RobotPose2D<double>                 mLastOdomPose;
    /* Accumulated travel distance since the last map update */
    double                              mAccumulatedTravelDist;
    /* Accumulated angle since the last map update */
    double                              mAccumulatedAngle;
    /* Odometry pose at the last map update */
    RobotPose2D<double>                 mLastMapUpdateOdomPose;
    /* Time of the last map update */
    double                              mLastMapUpdateTime;
    /* Map update threshold for accumulated travel distance */
    double                              mUpdateThresholdTravelDist;
    /* Map update threshold for accumulated angle */
    double                              mUpdateThresholdAngle;
    /* Map update threshold for the elapsed time since the last map update */
    double                              mUpdateThresholdTime;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_LIDAR_GRAPH_SLAM_HPP */
