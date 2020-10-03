
/* grid_map_builder.cpp */

#include <cmath>
#include <cstdlib>
#include <limits>
#include <map>
#include <vector>

#include "my_lidar_graph_slam/mapping/grid_map_builder.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/*
 * GridMapBuilder class implementations
 */

/* Constructor */
GridMapBuilder::GridMapBuilder(
    const double mapResolution,
    const int patchSize,
    const int numOfScansForLatestMap,
    const double travelDistThreshold,
    const double usableRangeMin,
    const double usableRangeMax,
    const double probHit,
    const double probMiss) :
    mResolution(mapResolution),
    mPatchSize(patchSize),
    mLatestMap(mapResolution, patchSize, 0, 0, Point2D<double>(0.0, 0.0)),
    mLatestMapPose(0.0, 0.0, 0.0),
    mAccumTravelDist(0.0),
    mNumOfScansForLatestMap(numOfScansForLatestMap),
    mLatestScanIdMin(0),
    mLatestScanIdMax(0),
    mLastRobotPose(0.0, 0.0, 0.0),
    mTravelDistLastLocalMap(0.0),
    mRobotPoseLastLocalMap(0.0, 0.0, 0.0),
    mTravelDistThreshold(travelDistThreshold),
    mUsableRangeMin(usableRangeMin),
    mUsableRangeMax(usableRangeMax),
    mProbHit(probHit),
    mProbMiss(probMiss)
{
}

/* Append the new scan data */
bool GridMapBuilder::AppendScan(
    LocalMapNodeMap& localMapNodes,
    const ScanNodeMap& scanNodes)
{
    /* Update the grid map */
    const bool localMapCreated = this->UpdateGridMap(localMapNodes, scanNodes);
    /* Update the grid map with latest scans */
    this->UpdateLatestMap(scanNodes);
    /* Return whether the new local map is created */
    return localMapCreated;
}

/* Re-create the local grid maps and latest map after the loop closure */
void GridMapBuilder::AfterLoopClosure(
    const LocalMapNodeMap& localMapNodes,
    const ScanNodeMap& scanNodes)
{
    /* Update the all local grid maps */
    for (auto& localMapData : this->mLocalMaps) {
        /* Retrieve the local map Id and global pose */
        const LocalMapId localMapId = localMapData.mId;
        const RobotPose2D<double>& globalMapPose =
            localMapNodes.At(localMapId).mGlobalPose;

        /* Reconstruct a local map */
        this->ConstructMapFromScans(
            globalMapPose, localMapData.mMap, scanNodes,
            localMapData.mScanNodeIdMin, localMapData.mScanNodeIdMax);

        /* Reset the precomputed grid maps */
        localMapData.mPrecomputedMaps.clear();
        localMapData.mPrecomputed = false;
    }

    /* Update the latest grid map with latest scans */
    this->UpdateLatestMap(scanNodes);

    /* Update the accumulated travel distance */
    this->UpdateAccumTravelDist(scanNodes);
}

/* Construct the global grid map */
void GridMapBuilder::ConstructGlobalMap(
    const ScanNodeMap& scanNodes,
    RobotPose2D<double>& globalMapPose,
    GridMapType& globalMap)
{
    /* Use all acquired scan data to build a global map */
    const NodeId scanNodeIdMin = scanNodes.NodeIdMin();
    const NodeId scanNodeIdMax = scanNodes.NodeIdMax();

    /* World coordinate pose of the latest scan node is used as the map pose */
    const RobotPose2D<double> mapPose = scanNodes.LatestNode().mGlobalPose;
    /* The above pose is the center of the local map coordinate */
    const Point2D<double> localCenterPos = Point2D<double>::Zero;

    /* Construct the global map */
    GridMapType gridMap { this->mResolution, this->mPatchSize,
                          0, 0, localCenterPos };
    this->ConstructMapFromScans(mapPose, gridMap, scanNodes,
                                scanNodeIdMin, scanNodeIdMax);

    /* Set the result */
    globalMapPose = mapPose;
    globalMap = std::move(gridMap);

    return;
}

/* Update the grid map (list of the local grid maps) */
bool GridMapBuilder::UpdateGridMap(
    LocalMapNodeMap& localMapNodes,
    const ScanNodeMap& scanNodes)
{
    /* Retrieve the latest scan node */
    const auto& latestScanNode = scanNodes.LatestNode();
    /* Current robot pose (stored in the latest scan node) */
    const RobotPose2D<double>& globalRobotPose = latestScanNode.mGlobalPose;
    /* Current scan data (stored in the latest scan node) */
    const auto& scanData = latestScanNode.mScanData;
    /* Current scan node Id */
    const NodeId scanNodeId = latestScanNode.mNodeId;

    /* Calculate the relative robot pose since the last method call */
    const RobotPose2D<double> relRobotPose = (this->mLocalMaps.size() == 0) ?
        RobotPose2D<double>(0.0, 0.0, 0.0) :
        InverseCompound(this->mLastRobotPose, globalRobotPose);

    this->mLastRobotPose = globalRobotPose;

    /* Update the accumulated travel distance */
    this->mAccumTravelDist += Distance(relRobotPose);
    /* Update the accumulated travel distance since the last grid map */
    this->mTravelDistLastLocalMap += Distance(relRobotPose);

    /* Determine whether to create a new local map */
    const bool travelDistThreshold =
        this->mTravelDistLastLocalMap >= this->mTravelDistThreshold;
    const bool isFirstScan = this->mLocalMaps.size() == 0;
    const bool createNewLocalMap = travelDistThreshold || isFirstScan;

    /* Create a new local map if necessary */
    if (createNewLocalMap) {
        /* The last local maps are marked as finished */
        if (this->mLocalMaps.size() > 0)
            this->mLocalMaps.back().mFinished = true;

        /* Determine the Id of the new local map */
        const LocalMapId localMapId = this->mLocalMaps.empty() ?
            LocalMapId { 0 } :
            LocalMapId { this->mLocalMaps.back().mId.mId + 1 };
        /* Insert a local map node to the pose graph */
        localMapNodes.Append(localMapId, globalRobotPose);

        /* Current robot pose in a world frame is used as the
         * origin of the local map coordinate */
        const Point2D<double> centerPos { 0.0, 0.0 };
        /* Create a new local map */
        GridMapType newLocalMap {
            this->mResolution, this->mPatchSize, 0, 0, centerPos };
        this->mLocalMaps.emplace_back(
            localMapId, std::move(newLocalMap), scanNodeId);

        /* Reset the variables properly */
        this->mTravelDistLastLocalMap = 0.0;
        this->mRobotPoseLastLocalMap = globalRobotPose;
    }

    /* Local map to which the latest scan is added */
    LocalMap& localMapData = this->mLocalMaps.back();
    GridMapType& localMap = localMapData.mMap;

    /* Local map pose in a world frame */
    const RobotPose2D<double>& globalMapPose =
        localMapNodes.LatestNode().mGlobalPose;

    /* Compute the scan points and bounding box */
    Point2D<double> localMinPos;
    Point2D<double> localMaxPos;
    std::vector<RobotPose2D<double>> localHitPoses;
    this->ComputeBoundingBoxAndScanPointsMapLocal(
        globalMapPose, globalRobotPose, scanData,
        localMinPos, localMaxPos, localHitPoses);

    /* Expand the local map so that it can contain the latest scan */
    localMap.Expand(localMinPos.mX, localMinPos.mY,
                    localMaxPos.mX, localMaxPos.mY);

    /* Compute the sensor pose from the robot pose */
    const RobotPose2D<double> globalSensorPose =
        Compound(globalRobotPose, scanData->RelativeSensorPose());
    /* Compute the sensor pose in a local map coordinate */
    const RobotPose2D<double> localSensorPose =
        InverseCompound(globalMapPose, globalSensorPose);
    /* Calculate the grid cell index corresponding to the sensor pose */
    const Point2D<int> sensorGridCellIdx =
        localMap.LocalPosToGridCellIndex(
            localSensorPose.mX, localSensorPose.mY);

    /* Integrate the scan into the local map */
    const std::size_t numOfFilteredScans = localHitPoses.size();

    for (std::size_t i = 0; i < numOfFilteredScans; ++i) {
        /* Compute the index of the hit grid cell */
        const Point2D<int> hitGridCellIdx =
            localMap.LocalPosToGridCellIndex(
                localHitPoses[i].mX, localHitPoses[i].mY);

        /* Compute the indices of the missed grid cells */
        const std::vector<Point2D<int>> missedGridCellIndices =
            this->ComputeMissedGridCellIndices(
                sensorGridCellIdx, hitGridCellIdx);

        /* Update occupancy probability values for missed grid cells */
        for (std::size_t j = 0; j < missedGridCellIndices.size(); ++j)
            localMap.Update(missedGridCellIndices[j], this->mProbMiss);

        /* Update occupancy probability values for hit grid cell */
        localMap.Update(hitGridCellIdx, this->mProbHit);
    }

    /* Update the index of the pose graph node */
    localMapData.mScanNodeIdMax = scanNodeId;

    /* Return whether the new local map is created */
    return createNewLocalMap;
}

/* Update the grid map with the latest scans */
void GridMapBuilder::UpdateLatestMap(const ScanNodeMap& scanNodes)
{
    /* Make sure that the pose graph is not empty */
    Assert(!scanNodes.Nodes().empty());

    /* Get the iterator pointing to the scan nodes used for latest maps */
    const int numOfScansForMap =
        std::min(1, this->mNumOfScansForLatestMap);
    const auto lastNodeIt = scanNodes.Nodes().rbegin();
    const auto latestNodeIt = std::next(lastNodeIt, numOfScansForMap);

    /* Calculate the minimum and maximum scan node Id */
    this->mLatestScanIdMin = latestNodeIt->first;
    this->mLatestScanIdMax = lastNodeIt->first;

    /* Update the pose of the latest map */
    this->mLatestMapPose = latestNodeIt->second.mGlobalPose;

    /* Update the latest map */
    this->ConstructMapFromScans(
        this->mLatestMapPose, this->mLatestMap, scanNodes,
        this->mLatestScanIdMin, this->mLatestScanIdMax);

    return;
}

/* Update the accumulated travel distance after the loop closure */
void GridMapBuilder::UpdateAccumTravelDist(const ScanNodeMap& scanNodes)
{
    /* Reset the accumulated travel distance */
    this->mAccumTravelDist = 0.0;

    /* Retrieve the two iterators pointing to the first scan node and
     * the node following to the last scan node */
    auto nodeIt = scanNodes.Nodes().begin();
    auto endIt = scanNodes.Nodes().end();
    auto nextIt = std::next(nodeIt);

    /* Return if the scan nodes are empty or only one scan node exists
     * `nextIt` is invalid and undefined behaviour if there is no node */
    if (nodeIt == endIt || nextIt == endIt)
        return;

    /* Accumulate the travel distance using pose graph nodes */
    for (; nextIt != endIt; ++nodeIt, ++nextIt) {
        const RobotPose2D<double>& nodePose = nodeIt->second.mGlobalPose;
        const RobotPose2D<double>& nextNodePose = nextIt->second.mGlobalPose;
        this->mAccumTravelDist += Distance(nodePose, nextNodePose);
    }
}

/* Construct the grid map from the specified scans */
void GridMapBuilder::ConstructMapFromScans(
    const RobotPose2D<double>& globalMapPose,
    GridMapType& gridMap,
    const ScanNodeMap& scanNodes,
    const NodeId scanNodeIdMin,
    const NodeId scanNodeIdMax) const
{
    /* Get the iterators to the scan nodes */
    auto firstNodeIt = scanNodes.LowerBound(scanNodeIdMin);
    auto lastNodeIt = scanNodes.UpperBound(scanNodeIdMax);

    /* Make sure that at least one scan data is used to build a map
     * Otherwise the bounding box below is not updated and causes a overflow */
    Assert(firstNodeIt != lastNodeIt);

    /* Compute the scan points and bounding box in a grid map frame */
    Point2D<double> localMinPos { std::numeric_limits<double>::max(),
                                 std::numeric_limits<double>::max() };
    Point2D<double> localMaxPos { std::numeric_limits<double>::min(),
                               std::numeric_limits<double>::min() };
    std::map<NodeId, std::vector<RobotPose2D<double>>> localHitPoses;

    for (auto nodeIt = firstNodeIt; nodeIt != lastNodeIt; ++nodeIt) {
        /* Retrieve the pose and scan data in the scan node */
        const NodeId scanNodeId = nodeIt->first;
        const auto& scanNode = nodeIt->second;
        const RobotPose2D<double>& globalNodePose = scanNode.mGlobalPose;
        const auto& scanData = scanNode.mScanData;

        /* Compute the global sensor pose from the node pose */
        const RobotPose2D<double> globalSensorPose =
            Compound(globalNodePose, scanData->RelativeSensorPose());
        /* Compute the local sensor pose */
        const RobotPose2D<double> localSensorPose =
            InverseCompound(globalMapPose, globalSensorPose);

        localMinPos.mX = std::min(localMinPos.mX, localSensorPose.mX);
        localMinPos.mY = std::min(localMinPos.mY, localSensorPose.mY);
        localMaxPos.mX = std::max(localMaxPos.mX, localSensorPose.mX);
        localMaxPos.mY = std::max(localMaxPos.mY, localSensorPose.mY);

        /* Compute the minimum and maximum range of the scan */
        const double minRange = std::max(
            this->mUsableRangeMin, scanData->MinRange());
        const double maxRange = std::min(
            this->mUsableRangeMax, scanData->MaxRange());

        /* Compute the scan points and bounding box */
        const std::size_t numOfScans = scanData->NumOfScans();
        std::vector<RobotPose2D<double>> nodeLocalHitPoses;
        nodeLocalHitPoses.reserve(numOfScans);

        for (std::size_t i = 0; i < numOfScans; ++i) {
            const double scanRange = scanData->RangeAt(i);

            if (scanRange >= maxRange || scanRange <= minRange)
                continue;

            /* Compute the hit point in a local frame */
            const RobotPose2D<double> globalHitPose =
                scanData->GlobalHitPose(globalSensorPose, i);
            const RobotPose2D<double> localHitPose =
                InverseCompound(globalMapPose, globalHitPose);
            nodeLocalHitPoses.push_back(localHitPose);

            /* Update the bounding box */
            localMinPos.mX = std::min(localMinPos.mX, localHitPose.mX);
            localMinPos.mY = std::min(localMinPos.mY, localHitPose.mY);
            localMaxPos.mX = std::max(localMaxPos.mX, localHitPose.mX);
            localMaxPos.mY = std::max(localMaxPos.mY, localHitPose.mY);
        }

        localHitPoses.emplace(scanNodeId, std::move(nodeLocalHitPoses));
    }

    /* Create a new grid map that contains all scan points */
    gridMap.Resize(localMinPos.mX, localMinPos.mY,
                   localMaxPos.mX, localMaxPos.mY);
    gridMap.Reset();

    /* Integrate the scan into the grid map
     * Reuse the same iterators as above since the pose graph is constant */
    for (auto nodeIt = firstNodeIt; nodeIt != lastNodeIt; ++nodeIt) {
        /* Retrieve the pose and scan data in the scan node */
        const NodeId scanNodeId = nodeIt->first;
        const auto& scanNode = nodeIt->second;
        const RobotPose2D<double>& globalNodePose = scanNode.mGlobalPose;
        const auto& scanData = scanNode.mScanData;

        /* Compute the global sensor pose from the node pose */
        const RobotPose2D<double> globalSensorPose =
            Compound(globalNodePose, scanData->RelativeSensorPose());
        /* Compute the local sensor pose */
        const RobotPose2D<double> localSensorPose =
            InverseCompound(globalMapPose, globalSensorPose);
        /* Compute the grid cell index corresponding to the sensor pose */
        const Point2D<int> sensorGridCellIdx =
            gridMap.LocalPosToGridCellIndex(
                localSensorPose.mX, localSensorPose.mY);

        /* Integrate the scan into the grid map */
        const std::size_t numOfFilteredScans = localHitPoses[scanNodeId].size();

        for (std::size_t i = 0; i < numOfFilteredScans; ++i) {
            /* Retrieve the hit point in a local frame */
            const RobotPose2D<double>& localHitPose =
                localHitPoses[scanNodeId].at(i);
            /* Compute the index of the hit grid cell */
            const Point2D<int> hitGridCellIdx =
                gridMap.LocalPosToGridCellIndex(
                    localHitPose.mX, localHitPose.mY);

            /* Compute the indices of the missed grid cells */
            const std::vector<Point2D<int>> missedGridCellIndices =
                this->ComputeMissedGridCellIndices(
                    sensorGridCellIdx, hitGridCellIdx);

            /* Update occupancy probability values for missed grid cells */
            for (std::size_t j = 0; j < missedGridCellIndices.size(); ++j)
                gridMap.Update(missedGridCellIndices[j], this->mProbMiss);

            /* Update occupancy probability values for hit grid cell */
            gridMap.Update(hitGridCellIdx, this->mProbHit);
        }
    }

    return;
}

/* Compute the bounding box of the scan and scan points in a local frame */
void GridMapBuilder::ComputeBoundingBoxAndScanPointsMapLocal(
    const RobotPose2D<double>& globalMapPose,
    const RobotPose2D<double>& globalRobotPose,
    const Sensor::ScanDataPtr<double>& scanData,
    Point2D<double>& localMinPos,
    Point2D<double>& localMaxPos,
    std::vector<RobotPose2D<double>>& localHitPoses)
{
    /* Compute the sensor pose from the robot pose */
    const RobotPose2D<double> globalSensorPose =
        Compound(globalRobotPose, scanData->RelativeSensorPose());

    /* Initialize the minimum coordinate of the given scan */
    localMinPos.mX = globalSensorPose.mX;
    localMinPos.mY = globalSensorPose.mY;

    /* Initialize the maximum coordinate of the given scan */
    localMaxPos.mX = globalSensorPose.mX;
    localMaxPos.mY = globalSensorPose.mY;

    const std::size_t numOfScans = scanData->NumOfScans();
    localHitPoses.reserve(numOfScans);

    /* Minimum and maximum range of the scan */
    const double minRange = std::max(
        this->mUsableRangeMin, scanData->MinRange());
    const double maxRange = std::min(
        this->mUsableRangeMax, scanData->MaxRange());

    /* Calculate the bounding box and scan points in a local frame */
    for (std::size_t i = 0; i < numOfScans; ++i) {
        const double scanRange = scanData->RangeAt(i);

        if (scanRange >= maxRange || scanRange <= minRange)
            continue;

        /* Calculate the hit point in a local frame */
        const RobotPose2D<double> globalHitPose =
            scanData->GlobalHitPose(globalSensorPose, i);
        const RobotPose2D<double> localHitPose =
            InverseCompound(globalMapPose, globalHitPose);
        localHitPoses.push_back(localHitPose);

        /* Update the corner positions (bounding box) */
        localMinPos.mX = std::min(localMinPos.mX, localHitPose.mX);
        localMinPos.mY = std::min(localMinPos.mY, localHitPose.mY);
        localMaxPos.mX = std::max(localMaxPos.mX, localHitPose.mX);
        localMaxPos.mY = std::max(localMaxPos.mY, localHitPose.mY);
    }

    return;
}

/* Compute the indices of the missed grid cells
 * using Bresenham algorithm */
std::vector<Point2D<int>> GridMapBuilder::ComputeMissedGridCellIndices(
    const Point2D<int>& startGridCellIdx,
    const Point2D<int>& endGridCellIdx) const
{
    /* Use Bresenham algorithm for computing indices */
    std::vector<Point2D<int>> gridCellIndices =
        Bresenham(startGridCellIdx, endGridCellIdx);

    /* Remove the last item since it is the hit grid cell */
    gridCellIndices.pop_back();

    return gridCellIndices;
}

/*
 * Utility function implementations
 */

/* Compute the maximum of a 'winSize' pixel wide row at each pixel */
void SlidingWindowMaxRow(const GridMapType& gridMap,
                         ConstMapType& intermediateMap,
                         const int winSize)
{
    /* Compute the maximum for each column */
    const double unknownVal = gridMap.UnknownValue();
    int colIdx = 0;

    std::function<double(int)> inFunc =
        [&colIdx, &gridMap, unknownVal](int rowIdx) {
        return gridMap.Value(colIdx, rowIdx, unknownVal);
    };

    std::function<void(int, double)> outFunc =
        [&colIdx, &gridMap, &intermediateMap, unknownVal]
        (int rowIdx, double maxVal) {
        const Point2D<int> patchIdx =
            gridMap.GridCellIndexToPatchIndex(colIdx, rowIdx);
        const bool isAllocated = gridMap.PatchIsAllocated(patchIdx);

        if (isAllocated || maxVal != unknownVal)
            intermediateMap.Update(colIdx, rowIdx, maxVal);
    };

    const int numOfGridCellsX = gridMap.NumOfGridCellsX();
    const int numOfGridCellsY = gridMap.NumOfGridCellsY();

    /* Apply the sliding window maximum function */
    for (colIdx = 0; colIdx < numOfGridCellsX; ++colIdx)
        SlidingWindowMax(inFunc, outFunc, numOfGridCellsY, winSize);
}

/* Compute the maximum of a 'winSize' pixel wide column at each pixel */
void SlidingWindowMaxCol(const ConstMapType& intermediateMap,
                         ConstMapType& precompMap,
                         const int winSize)
{
    /* Compute the maximum for each row */
    const double unknownVal = intermediateMap.UnknownValue();
    int rowIdx = 0;

    std::function<double(int)> inFunc =
        [&rowIdx, &intermediateMap, unknownVal](int colIdx) {
        return intermediateMap.Value(colIdx, rowIdx, unknownVal);
    };

    std::function<void(int, double)> outFunc =
        [&rowIdx, &intermediateMap, &precompMap, unknownVal]
        (int colIdx, double maxVal) {
        const Point2D<int> patchIdx =
            intermediateMap.GridCellIndexToPatchIndex(colIdx, rowIdx);
        const bool isAllocated = intermediateMap.PatchIsAllocated(patchIdx);

        if (isAllocated || maxVal != unknownVal)
            precompMap.Update(colIdx, rowIdx, maxVal);
    };

    const int numOfGridCellsX = intermediateMap.NumOfGridCellsX();
    const int numOfGridCellsY = intermediateMap.NumOfGridCellsY();

    /* Apply the sliding window maximum function */
    for (rowIdx = 0; rowIdx < numOfGridCellsY; ++rowIdx)
        SlidingWindowMax(inFunc, outFunc, numOfGridCellsX, winSize);
}

/* Precompute coarser grid maps for efficiency */
void PrecomputeGridMaps(const GridMapType& gridMap,
                        std::map<int, ConstMapType>& precomputedMaps,
                        const int nodeHeightMax)
{
    /* Create the temporary grid map to store the intermediate result
     * The map size is as same as the local grid map and is reused for
     * several times below */
    ConstMapType intermediateMap = ConstMapType::CreateSameSizeMap(gridMap);

    /* Clear the precomputed coarser grid maps */
    precomputedMaps.clear();

    /* Compute a grid map for each node height */
    for (int nodeHeight = 0, winSize = 1;
         nodeHeight <= nodeHeightMax; ++nodeHeight, winSize <<= 1) {
        /* Precompute a grid map */
        ConstMapType precompMap =
            PrecomputeGridMap(gridMap, intermediateMap, winSize);

        /* Append the newly created map */
        precomputedMaps.emplace(nodeHeight, std::move(precompMap));
    }
}

/* Precompute grid map for efficiency */
ConstMapType PrecomputeGridMap(const GridMapType& gridMap,
                               ConstMapType& intermediateMap,
                               const int winSize)
{
    /* Create a new grid map
     * Each pixel stores the maximum of the occupancy probability values of
     * 'winSize' * 'winSize' box of pixels beginning there */
    ConstMapType precompMap = ConstMapType::CreateSameSizeMap(gridMap);

    /* Store the maximum of the 'winSize' pixel wide row */
    SlidingWindowMaxRow(gridMap, intermediateMap, winSize);
    /* Store the maximum of the 'winSize' pixel wide column */
    SlidingWindowMaxCol(intermediateMap, precompMap, winSize);

    return precompMap;
}

/* Precompute grid map for efficiency */
ConstMapType PrecomputeGridMap(const GridMapType& gridMap,
                               const int winSize)
{
    /* Create a temporary map to store the intermediate result */
    ConstMapType intermediateMap = ConstMapType::CreateSameSizeMap(gridMap);

    /* Create a new grid map */
    ConstMapType precompMap = ConstMapType::CreateSameSizeMap(gridMap);

    /* Store the maximum of the 'winSize' pixel wide row */
    SlidingWindowMaxRow(gridMap, intermediateMap, winSize);
    /* Store the maximum of the 'winSize' pixel wide column */
    SlidingWindowMaxCol(intermediateMap, precompMap, winSize);

    return precompMap;
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
