
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
using ConstMapType = GridMap<ConstGridCell<double>>;

/*
 * LocalMapPosition struct stores information about the local map bounding box,
 * Ids of the scan nodes that reside in this local map, which are necessary
 * for the loop detection candidate search
 */
struct LocalMapPosition
{
    /* Constructor */
    LocalMapPosition(const LocalMapId localMapId,
                     const Point2D<double>& minGlobalPos,
                     const Point2D<double>& maxGlobalPos,
                     const NodeId scanNodeIdMin,
                     const NodeId scanNodeIdMax,
                     const bool isFinished) :
        mId(localMapId),
        mMinGlobalPos(minGlobalPos),
        mMaxGlobalPos(maxGlobalPos),
        mScanNodeIdMin(scanNodeIdMin),
        mScanNodeIdMax(scanNodeIdMax),
        mFinished(isFinished) { }

    /* Destructor */
    ~LocalMapPosition() = default;

    /* Local map Id */
    const LocalMapId      mId;
    /* Minimum position of the local map (in a world frame) */
    const Point2D<double> mMinGlobalPos;
    /* Maximum position of the local map (in a world frame) */
    const Point2D<double> mMaxGlobalPos;
    /* Minimum scan node Id inside this local map */
    const NodeId          mScanNodeIdMin;
    /* Maximum scan node Id inside this local map */
    const NodeId          mScanNodeIdMax;
    /* Flag to represent whether this local map is finished */
    const bool            mFinished;
};

/*
 * LocalMapInfo struct keeps necessary information for a local grid map
 * A single local grid map consists of the sequence of scan data
 * within the range from `mScanNodeIdMin` to `mScanNodeIdMax` and has
 * an associated local map Id `mId`, from which the local map pose in a
 * world coordinate is obtained using the pose graph
 */
struct LocalMapInfo final
{
    /* Constructor */
    LocalMapInfo(const LocalMapId localMapId,
                 GridMapType&& gridMap,
                 const NodeId scanNodeId) :
        mId(localMapId),
        mMap(std::move(gridMap)),
        mScanNodeIdMin(scanNodeId),
        mScanNodeIdMax(scanNodeId),
        mFinished(false),
        mPrecomputed(false) { }

    /* Destructor */
    ~LocalMapInfo() = default;

    /* Copy constructor */
    LocalMapInfo(const LocalMapInfo&) = default;
    /* Copy assignment operator */
    LocalMapInfo& operator=(const LocalMapInfo&) = default;
    /* Move constructor */
    LocalMapInfo(LocalMapInfo&&) noexcept = default;
    /* Move assignment operator */
    LocalMapInfo& operator=(LocalMapInfo&&) noexcept = default;

    /* Local map Id */
    const LocalMapId            mId;
    /* Local grid map consisting of the sequence of the scan data
     * from `mScanNodeIdMin` to `mScanNodeIdMax` */
    GridMapType                 mMap;
    /* Minimum scan node Id */
    NodeId                      mScanNodeIdMin;
    /* Maximum scan node Id */
    NodeId                      mScanNodeIdMax;
    /* Flags to represent whether this local map is finished and will not
     * be changed (no more scan data is added) */
    bool                        mFinished;
    /* Flags to represent whether the coarser grid maps for a loop detection
     * are precomputed */
    bool                        mPrecomputed;
    /* Map of precomputed grid maps for a loop detection */
    std::map<int, ConstMapType> mPrecomputedMaps;
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
    const GridMapType& gridMap,
    std::map<int, PrecomputedMapType>& precompMaps,
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
