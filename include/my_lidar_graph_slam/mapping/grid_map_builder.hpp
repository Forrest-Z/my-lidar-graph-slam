
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
 * LocalMapData struct stores information about the local map bounding box,
 * Ids of the scan nodes that reside in this local map, which are necessary
 * for the loop detection candidate search
 */
struct LocalMapData final
{
    /* Constructor */
    LocalMapData(const LocalMapId localMapId,
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
    ~LocalMapData() = default;

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
 * LocalMap struct keeps necessary information for a local grid map
 * A single local grid map consists of the sequence of scan data
 * within the range from `mScanNodeIdMin` to `mScanNodeIdMax` and has
 * an associated local map Id `mId`, from which the local map pose in a
 * world coordinate is obtained using the pose graph
 */
struct LocalMap final
{
    /* Constructor */
    LocalMap(const LocalMapId localMapId,
             GridMapType&& gridMap,
             const NodeId scanNodeId) :
        mId(localMapId),
        mMap(std::move(gridMap)),
        mScanNodeIdMin(scanNodeId),
        mScanNodeIdMax(scanNodeId),
        mFinished(false),
        mPrecomputed(false) { }

    /* Destructor */
    ~LocalMap() = default;

    /* Copy constructor */
    LocalMap(const LocalMap&) = default;
    /* Copy assignment operator */
    LocalMap& operator=(const LocalMap&) = default;
    /* Move constructor */
    LocalMap(LocalMap&&) noexcept = default;
    /* Move assignment operator */
    LocalMap& operator=(LocalMap&&) noexcept = default;

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

/*
 * GridMapBuilder class is responsible for creating and updating grid maps
 */
class GridMapBuilder
{
public:
    /* Constructor */
    GridMapBuilder(const double mapResolution,
                   const int patchSize,
                   const int numOfScansForLatestMap,
                   const double travelDistThreshold,
                   const double usableRangeMin,
                   const double usableRangeMax,
                   const double probHit,
                   const double probMiss);

    /* Destructor */
    ~GridMapBuilder() = default;

    /* Append the new scan data */
    bool AppendScan(LocalMapNodeMap& localMapNodes,
                    const ScanNodeMap& scanNodes);

    /* Re-create the local grid maps and latest map after the loop closure */
    void AfterLoopClosure(const LocalMapNodeMap& localMapNodes,
                          const ScanNodeMap& scanNodes);

    /* Construct the global map */
    void ConstructGlobalMap(
        const ScanNodeMap& scanNodes,
        RobotPose2D<double>& globalMapPose,
        GridMapType& globalMap);

    /* Retrieve the local grid maps */
    inline const std::vector<LocalMap>& LocalMaps() const
    { return this->mLocalMaps; }

    /* Retrieve the local map information of the specified index */
    inline LocalMap& LocalMapAt(int localMapIdx)
    { return this->mLocalMaps.at(localMapIdx); }
    /* Retrieve the local map information of the specified index */
    inline const LocalMap& LocalMapAt(int localMapIdx) const
    { return this->mLocalMaps.at(localMapIdx); }

    /* Retrieve the grid map constructed from the latest scans */
    inline const GridMapType& LatestMap() const { return this->mLatestMap; }

    /* Get the accumulated travel distance */
    inline double AccumTravelDist() const { return this->mAccumTravelDist; }

    /* Get the minimum Id of the latest scan nodes */
    inline NodeId LatestScanIdMin() const { return this->mLatestScanIdMin; }
    /* Get the maximum Id of the latest scan nodes */
    inline NodeId LatestScanIdMax() const { return this->mLatestScanIdMax; }

private:
    /* Update the grid map (list of the local grid maps) */
    bool UpdateGridMap(LocalMapNodeMap& localMapNodes,
                       const ScanNodeMap& scanNodes);

    /* Update the grid map with the latest scans */
    void UpdateLatestMap(const ScanNodeMap& scanNodes);

    /* Update the accumulated travel distance after the loop closure */
    void UpdateAccumTravelDist(const ScanNodeMap& scanNodes);

    /* Construct the grid map from the specified scans */
    void ConstructMapFromScans(
        const RobotPose2D<double>& globalMapPose,
        GridMapType& gridMap,
        const ScanNodeMap& scanNodes,
        const NodeId scanNodeIdMin,
        const NodeId scanNodeIdMax) const;

    /* Compute the bounding box of the scan and scan points in a local frame */
    void ComputeBoundingBoxAndScanPointsMapLocal(
        const RobotPose2D<double>& globalMapPose,
        const RobotPose2D<double>& globalRobotPose,
        const Sensor::ScanDataPtr<double>& scanData,
        Point2D<double>& localMinPos,
        Point2D<double>& localMaxPos,
        std::vector<Point2D<double>>& localHitPoints);

    /* Compute the indices of the missed grid cells
     * using Bresenham algorithm */
    std::vector<Point2D<int>> ComputeMissedGridCellIndices(
        const Point2D<int>& startGridCellIdx,
        const Point2D<int>& endGridCellIdx) const;

private:
    /* Map resolution (in meters) */
    const double          mResolution;
    /* Patch size (in the number of grid cells) */
    const int             mPatchSize;
    /* Vector of the local grid maps */
    std::vector<LocalMap> mLocalMaps;
    /* Grid map constructed from the latest scans
     * Used for scan matching and updated when a new scan data is available */
    GridMapType           mLatestMap;
    /* Latest map pose in a world frame */
    RobotPose2D<double>   mLatestMapPose;
    /* Accumulated travel distance */
    double                mAccumTravelDist;
    /* The number of scans used to construct the latest map */
    const int             mNumOfScansForLatestMap;
    /* The minimum Id of the latest scan nodes */
    NodeId                mLatestScanIdMin;
    /* The maximum Id of the latest scan nodes */
    NodeId                mLatestScanIdMax;
    /* Last robot pose in a world frame */
    RobotPose2D<double>   mLastRobotPose;
    /* Accumulated travel distance since the last local grid map is created */
    double                mTravelDistLastLocalMap;
    /* Robot pose in a world frame when the last local grid map is created */
    RobotPose2D<double>   mRobotPoseLastLocalMap;
    /* Travel distance threshold for creating a new local grid map */
    const double          mTravelDistThreshold;
    /* Minimum range of the laser scan that is considered valid */
    const double          mUsableRangeMin;
    /* Maximum range of the laser scan that is considered valid */
    const double          mUsableRangeMax;
    /* Occupancy probability value for hit grid cell
     * Used for calculating the probability value with Binary Bayes Filter */
    const double          mProbHit;
    /* Occupancy probability value for missed grid cell
     * Used for calculating the probability value with Binary Bayes Filter */
    const double          mProbMiss;
};

/*
 * Utility function declarations
 */

/* Compute the maximum of a 'winSize' pixel wide row at each pixel */
void SlidingWindowMaxRow(const GridMapType& gridMap,
                         ConstMapType& intermediateMap,
                         const int winSize);

/* Compute the maximum of a 'winSize' pixel wide column at each pixel */
void SlidingWindowMaxCol(const ConstMapType& intermediateMap,
                         ConstMapType& precompMap,
                         const int winSize);

/* Precompute coarser grid maps for efficiency */
void PrecomputeGridMaps(const GridMapType& gridMap,
                        std::map<int, ConstMapType>& precompMaps,
                        const int nodeHeightMax);

/* Precompute grid map for efficiency */
ConstMapType PrecomputeGridMap(const GridMapType& gridMap,
                               ConstMapType& intermediateMap,
                               const int winSize);

/* Precompute grid map for efficiency */
ConstMapType PrecomputeGridMap(const GridMapType& gridMap,
                               const int winSize);

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_GRID_MAP_GRID_MAP_BUILDER_HPP */
