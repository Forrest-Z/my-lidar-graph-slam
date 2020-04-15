
/* lidar_graph_slam.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_LIDAR_GRAPH_SLAM_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_LIDAR_GRAPH_SLAM_HPP

#include <memory>
#include <utility>

#include "my_lidar_graph_slam/pose.hpp"
#include "my_lidar_graph_slam/grid_map/counting_grid_cell.hpp"
#include "my_lidar_graph_slam/mapping/grid_map_builder.hpp"
#include "my_lidar_graph_slam/mapping/pose_graph.hpp"
#include "my_lidar_graph_slam/mapping/scan_matcher.hpp"
#include "my_lidar_graph_slam/sensor/sensor_data.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

class LidarGraphSlam
{
public:
    /* Type definitions */
    using ScanType = Sensor::ScanDataPtr<double>;
    using GridMapType = GridMap<CountingGridCell<double>>;
    using ScanMatcherType = ScanMatcher<GridMapType, ScanType>;

    /* Constructor */
    LidarGraphSlam(std::unique_ptr<ScanMatcherType>&& scanMatcher,
                   double mapResolution,
                   int patchSize,
                   int numOfScansForLocalMap,
                   double travelDistThresholdForLocalMap,
                   const RobotPose2D<double>& initialPose,
                   double updateThresholdTravelDist,
                   double updateThresholdAngle,
                   double updateThresholdTime,
                   double usableRangeMin,
                   double usableRangeMax);

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
    bool ProcessScan(const ScanType& scanData,
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
    int                              mProcessCount;
    /* Grid map */
    std::shared_ptr<GridMapBuilder>  mGridMapBuilder;
    /* Pose graph */
    std::shared_ptr<PoseGraph>       mPoseGraph;
    /* Scan matcher */
    std::unique_ptr<ScanMatcherType> mScanMatcher;
    /* Initial pose */
    RobotPose2D<double>              mInitialPose;
    /* Odometry pose at the last map update */
    RobotPose2D<double>              mLastMapUpdateOdomPose;
    /* Time of the last map update */
    double                           mLastMapUpdateTime;
    /* Map update threshold for accumulated travel distance */
    double                           mUpdateThresholdTravelDist;
    /* Map update threshold for accumulated angle */
    double                           mUpdateThresholdAngle;
    /* Map update threshold for the elapsed time since the last map update */
    double                           mUpdateThresholdTime;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_LIDAR_GRAPH_SLAM_HPP */