
/* grid_map_builder.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_GRID_MAP_GRID_MAP_BUILDER_HPP
#define MY_LIDAR_GRAPH_SLAM_GRID_MAP_GRID_MAP_BUILDER_HPP

#include <vector>

#include "my_lidar_graph_slam/point.hpp"
#include "my_lidar_graph_slam/pose.hpp"
#include "my_lidar_graph_slam/grid_map/binary_bayes_grid_cell.hpp"
#include "my_lidar_graph_slam/grid_map/grid_map.hpp"
#include "my_lidar_graph_slam/mapping/pose_graph.hpp"
#include "my_lidar_graph_slam/sensor/sensor_data.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

class GridMapBuilder
{
public:
    /* Type definitions */
    using GridMapType = GridMap<BinaryBayesGridCell<double>>;
    using PatchType = typename GridMapType::PatchType;
    using ScanType = Sensor::ScanDataPtr<double>;

public:
    /*
     * LocalMapInfo struct binds grid maps and pose graphs
     */
    struct LocalMapInfo
    {
        /* Constructor */
        LocalMapInfo(GridMapType&& gridMap,
                     int poseGraphNodeIdx) :
            mMap(std::move(gridMap)),
            mPoseGraphNodeIdxMin(poseGraphNodeIdx),
            mPoseGraphNodeIdxMax(poseGraphNodeIdx) { }
        
        /* Destructor */
        ~LocalMapInfo() = default;

        /* Copy constructor (disabled) */
        LocalMapInfo(const LocalMapInfo&) = delete;
        /* Copy assignment operator (disabled) */
        LocalMapInfo& operator=(const LocalMapInfo&) = delete;

        /* Move constructor */
        LocalMapInfo(LocalMapInfo&& other) noexcept;
        /* Move assignment operator */
        LocalMapInfo& operator=(LocalMapInfo&& other) noexcept;

        /* Local grid map, which consists of the scan data
         * from the pose graph nodes within the range of
         * [mPoseGraphNodeIdxMin, mPoseGraphNodeIdxMax] */
        GridMapType mMap;
        /* Minimum index of the pose graph node */
        int          mPoseGraphNodeIdxMin;
        /* Maximum index of the pose graph node */
        int          mPoseGraphNodeIdxMax;
    };

public:
    /* Constructor */
    GridMapBuilder(double mapResolution,
                   int patchSize,
                   int numOfScansForLatestMap,
                   double travelDistThreshold,
                   double usableRangeMin,
                   double usableRangeMax,
                   double probHit,
                   double probMiss);

    /* Destructor */
    ~GridMapBuilder() = default;

    /* Append the new scan data */
    void AppendScan(const std::shared_ptr<PoseGraph>& poseGraph);

    /* Re-create the local grid maps and latest map after the loop closure */
    void AfterLoopClosure(const std::shared_ptr<PoseGraph>& poseGraph);

    /* Construct the global map */
    GridMapType ConstructGlobalMap(
        const std::shared_ptr<PoseGraph>& poseGraph) const;
    
    /* Retrieve the local grid maps */
    inline const std::vector<LocalMapInfo>& LocalMaps() const
    { return this->mLocalMaps; }

    /* Retrieve the local map information of the specified index */
    inline const LocalMapInfo& LocalMapAt(int localMapIdx) const
    { return this->mLocalMaps.at(localMapIdx); }

    /* Retrieve the grid map constructed from the latest scans */
    inline const GridMapType& LatestMap() const { return this->mLatestMap; }

    /* Get the minimum index of the latest scans */
    inline int LatestScanIdxMin() const { return this->mLatestScanIdxMin; }
    /* Get the maximum index of the latest scans */
    inline int LatestScanIdxMax() const { return this->mLatestScanIdxMax; }

private:
    /* Update the grid map (list of the local grid maps) */
    void UpdateGridMap(
        const std::shared_ptr<PoseGraph>& poseGraph);
    
    /* Update the grid map with the latest scans */
    void UpdateLatestMap(
        const std::shared_ptr<PoseGraph>& poseGraph);

    /* Construct the grid map from the specified scans */
    void ConstructMapFromScans(
        GridMapType& gridMap,
        const std::shared_ptr<PoseGraph>& poseGraph,
        int poseGraphNodeIdxMin,
        int poseGraphNodeIdxMax) const;

    /* Compute the bounding box of the scan and scan points */
    void ComputeBoundingBoxAndScanPoints(
        const RobotPose2D<double>& robotPose,
        const ScanType& scanData,
        Point2D<double>& bottomLeft,
        Point2D<double>& topRight,
        std::vector<Point2D<double>>& hitPoints);

    /* Compute the indices of the missed grid cells
     * using Bresenham algorithm */
    std::vector<Point2D<int>> ComputeMissedGridCellIndices(
        const Point2D<int>& startGridCellIdx,
        const Point2D<int>& endGridCellIdx) const;

private:
    /* Map resolution (in meters) */
    double                        mMapResolution;
    /* Patch size (in the number of grid cells) */
    int                           mPatchSize;
    /* List of the local grid maps */
    std::vector<LocalMapInfo>     mLocalMaps;
    /* Grid map constructed from the latest scans
     * Used for scan matching and updated when new scan data is available */
    GridMapType                   mLatestMap;
    /* The number of scans used to construct the latest map */
    int                           mNumOfScansForLatestMap;
    /* The minimum index of the latest scans */
    int                           mLatestScanIdxMin;
    /* The maximum index of the latest scans */
    int                           mLatestScanIdxMax;
    /* Last robot pose */
    RobotPose2D<double>           mLastRobotPose;
    /* Accumulated travel distance since the last local grid map is created */
    double                        mAccumulatedTravelDist;
    /* Robot pose when the last local grid map is created */
    RobotPose2D<double>           mRobotPoseLastLocalMap;
    /* Travel distance threshold for creating a new local grid map */
    double                        mTravelDistThreshold;
    /* Minimum range of the laser scan that is considered valid */
    double                        mUsableRangeMin;
    /* Maximum range of the laser scan that is considered valid */
    double                        mUsableRangeMax;
    /* Occupancy probability value for hit grid cell
     * Used for calculating the probability value with Binary Bayes Filter */
    double                        mProbHit;
    /* Occupancy probability value for missed grid cell
     * Used for calculating the probability value with Binary Bayes Filter */
    double                        mProbMiss;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_GRID_MAP_GRID_MAP_BUILDER_HPP */
