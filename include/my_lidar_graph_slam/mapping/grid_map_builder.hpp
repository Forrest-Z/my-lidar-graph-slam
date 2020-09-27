
/* grid_map_builder.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_GRID_MAP_GRID_MAP_BUILDER_HPP
#define MY_LIDAR_GRAPH_SLAM_GRID_MAP_GRID_MAP_BUILDER_HPP

#include <functional>
#include <map>
#include <vector>

#include "my_lidar_graph_slam/point.hpp"
#include "my_lidar_graph_slam/pose.hpp"
#include "my_lidar_graph_slam/grid_map/binary_bayes_grid_cell.hpp"
#include "my_lidar_graph_slam/grid_map/const_grid_cell.hpp"
#include "my_lidar_graph_slam/grid_map/grid_map.hpp"
#include "my_lidar_graph_slam/mapping/pose_graph.hpp"
#include "my_lidar_graph_slam/sensor/sensor_data.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/* Type definitions */
using GridMapType = GridMap<BinaryBayesGridCell<double>>;
using PrecomputedMapType = GridMap<ConstGridCell<double>>;

/*
 * LocalMapPosition struct stores information about the local map bounding box,
 * indices of the pose graph nodes that reside in this local map, which are
 * necessary for the loop detection candidate search
 */
struct LocalMapPosition
{
    /* Constructor */
    LocalMapPosition(const Point2D<double>& minPos,
                     const Point2D<double>& maxPos,
                     const int poseGraphNodeIdxMin,
                     const int poseGraphNodeIdxMax,
                     const bool isFinished) :
        mMinPos(minPos),
        mMaxPos(maxPos),
        mPoseGraphNodeIdxMin(poseGraphNodeIdxMin),
        mPoseGraphNodeIdxMax(poseGraphNodeIdxMax),
        mFinished(isFinished) { }
    /* Destructor */
    ~LocalMapPosition() = default;

    /* Minimum position of the local map (in world frame) */
    Point2D<double> mMinPos;
    /* Maximum position of the local map (in world frame) */
    Point2D<double> mMaxPos;
    /* Minimum index of the pose graph node inside this local map */
    int             mPoseGraphNodeIdxMin;
    /* Maximum index of the pose graph node inside this local map */
    int             mPoseGraphNodeIdxMax;
    /* Flag to determine whether the grid map is in finished state */
    bool            mFinished;
};

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
        mPoseGraphNodeIdxMax(poseGraphNodeIdx),
        mFinished(false),
        mPrecomputed(false) { }

    /* Destructor */
    ~LocalMapInfo() = default;

    /* Copy constructor */
    LocalMapInfo(const LocalMapInfo& other) = default;
    /* Copy assignment operator */
    LocalMapInfo& operator=(const LocalMapInfo& other) = default;
    /* Move constructor */
    LocalMapInfo(LocalMapInfo&& other) noexcept = default;
    /* Move assignment operator */
    LocalMapInfo& operator=(LocalMapInfo&& other) noexcept = default;

    /* Local grid map, which consists of the scan data
     * from the pose graph nodes within the range of
     * [mPoseGraphNodeIdxMin, mPoseGraphNodeIdxMax] */
    GridMapType                       mMap;
    /* Minimum index of the pose graph node */
    int                               mPoseGraphNodeIdxMin;
    /* Maximum index of the pose graph node */
    int                               mPoseGraphNodeIdxMax;
    /* Flags to determine whether the grid maps are finished and
     * will not be changed (no more new scan data will be added) */
    bool                              mFinished;
    /* Flags to determine whether the grid maps for the loop detection are
     * already precomputed */
    bool                              mPrecomputed;
    /* List of precomputed grid maps for loop detection */
    std::map<int, PrecomputedMapType> mPrecomputedMaps;
};

class GridMapBuilder
{
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
    bool AppendScan(const std::shared_ptr<PoseGraph>& poseGraph);

    /* Re-create the local grid maps and latest map after the loop closure */
    void AfterLoopClosure(const std::shared_ptr<PoseGraph>& poseGraph);

    /* Construct the global map */
    GridMapType ConstructGlobalMap(
        const std::shared_ptr<PoseGraph>& poseGraph) const;

    /* Retrieve the local grid maps */
    inline const std::vector<LocalMapInfo>& LocalMaps() const
    { return this->mLocalMaps; }

    /* Retrieve the local map information of the specified index */
    inline LocalMapInfo& LocalMapAt(int localMapIdx)
    { return this->mLocalMaps.at(localMapIdx); }
    /* Retrieve the local map information of the specified index */
    inline const LocalMapInfo& LocalMapAt(int localMapIdx) const
    { return this->mLocalMaps.at(localMapIdx); }

    /* Retrieve the grid map constructed from the latest scans */
    inline const GridMapType& LatestMap() const { return this->mLatestMap; }

    /* Get the accumulated travel distance */
    inline double AccumTravelDist() const { return this->mAccumTravelDist; }

    /* Get the minimum index of the latest scans */
    inline int LatestScanIdxMin() const { return this->mLatestScanIdxMin; }
    /* Get the maximum index of the latest scans */
    inline int LatestScanIdxMax() const { return this->mLatestScanIdxMax; }

private:
    /* Update the grid map (list of the local grid maps) */
    bool UpdateGridMap(
        const std::shared_ptr<PoseGraph>& poseGraph);
    
    /* Update the grid map with the latest scans */
    void UpdateLatestMap(
        const std::shared_ptr<PoseGraph>& poseGraph);
    
    /* Update the accumulated travel distance after the loop closure */
    void UpdateAccumTravelDist(
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
        const Sensor::ScanDataPtr<double>& scanData,
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
    double                        mResolution;
    /* Patch size (in the number of grid cells) */
    int                           mPatchSize;
    /* List of the local grid maps */
    std::vector<LocalMapInfo>     mLocalMaps;
    /* Grid map constructed from the latest scans
     * Used for scan matching and updated when new scan data is available */
    GridMapType                   mLatestMap;
    /* Accumulated travel distance */
    double                        mAccumTravelDist;
    /* The number of scans used to construct the latest map */
    int                           mNumOfScansForLatestMap;
    /* The minimum index of the latest scans */
    int                           mLatestScanIdxMin;
    /* The maximum index of the latest scans */
    int                           mLatestScanIdxMax;
    /* Last robot pose */
    RobotPose2D<double>           mLastRobotPose;
    /* Accumulated travel distance since the last local grid map is created */
    double                        mTravelDistLastLocalMap;
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

/*
 * Utility function declarations
 */

/* Compute the maximum of a 'winSize' pixel wide row at each pixel */
void SlidingWindowMaxRow(
    const GridMapType& gridMap,
    PrecomputedMapType& intermediateMap,
    int winSize);

/* Compute the maximum of a 'winSize' pixel wide column at each pixel */
void SlidingWindowMaxCol(
    const PrecomputedMapType& intermediateMap,
    PrecomputedMapType& precompMap,
    int winSize);

/* Precompute coarser grid maps for efficiency */
void PrecomputeGridMaps(
    LocalMapInfo& localMapInfo,
    const int nodeHeightMax);

/* Precompute grid map for efficiency */
PrecomputedMapType PrecomputeGridMap(
    const GridMapType& gridMap,
    PrecomputedMapType& intermediateMap,
    int winSize);

/* Precompute grid map for efficiency */
PrecomputedMapType PrecomputeGridMap(
    const GridMapType& gridMap,
    int winSize);

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_GRID_MAP_GRID_MAP_BUILDER_HPP */
