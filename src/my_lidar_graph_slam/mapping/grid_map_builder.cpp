
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
 * LocalMapInfo struct implementations
 */

/* Move constructor */
GridMapBuilder::LocalMapInfo::LocalMapInfo(
    GridMapBuilder::LocalMapInfo&& other) noexcept:
    mMap(std::move(other.mMap)),
    mPoseGraphNodeIdxMin(other.mPoseGraphNodeIdxMin),
    mPoseGraphNodeIdxMax(other.mPoseGraphNodeIdxMax),
    mFinished(other.mFinished),
    mPrecomputed(other.mPrecomputed),
    mPrecomputedMaps(std::move(other.mPrecomputedMaps))
{
}

/* Move assignment operator */
GridMapBuilder::LocalMapInfo&
    GridMapBuilder::LocalMapInfo::operator=(
    GridMapBuilder::LocalMapInfo&& other) noexcept
{
    if (this == &other)
        return *this;
    
    this->mMap = std::move(other.mMap);
    this->mPoseGraphNodeIdxMin = other.mPoseGraphNodeIdxMin;
    this->mPoseGraphNodeIdxMax = other.mPoseGraphNodeIdxMax;
    this->mFinished = other.mFinished;
    this->mPrecomputed = other.mPrecomputed;
    this->mPrecomputedMaps = std::move(other.mPrecomputedMaps);

    return *this;
}

/*
 * GridMapBuilder class implementations
 */

/* Constructor */
GridMapBuilder::GridMapBuilder(
    double mapResolution,
    int patchSize,
    int numOfScansForLatestMap,
    double travelDistThreshold,
    double usableRangeMin,
    double usableRangeMax,
    double probHit,
    double probMiss) :
    mMapResolution(mapResolution),
    mPatchSize(patchSize),
    mLatestMap(mapResolution, patchSize, 0, 0, Point2D<double>(0.0, 0.0)),
    mAccumTravelDist(0.0),
    mNumOfScansForLatestMap(numOfScansForLatestMap),
    mLatestScanIdxMin(0),
    mLatestScanIdxMax(0),
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
void GridMapBuilder::AppendScan(
    const std::shared_ptr<PoseGraph>& poseGraph)
{
    /* Update the grid map */
    this->UpdateGridMap(poseGraph);

    /* Update the grid map with latest scans */
    this->UpdateLatestMap(poseGraph);
}

/* Re-create the local grid maps and latest map after the loop closure */
void GridMapBuilder::AfterLoopClosure(
    const std::shared_ptr<PoseGraph>& poseGraph)
{
    /* Update the all local grid maps */
    for (auto& localMapInfo : this->mLocalMaps) {
        this->ConstructMapFromScans(localMapInfo.mMap, poseGraph,
                                    localMapInfo.mPoseGraphNodeIdxMin,
                                    localMapInfo.mPoseGraphNodeIdxMax);
        /* Reset the precomputed grid maps */
        localMapInfo.mPrecomputedMaps.clear();
        localMapInfo.mPrecomputed = false;
    }

    /* Update the grid map with latest scans */
    this->UpdateLatestMap(poseGraph);

    /* Update the accumulated travel distance */
    this->UpdateAccumTravelDist(poseGraph);
}

/* Construct the global grid map */
GridMapBuilder::GridMapType GridMapBuilder::ConstructGlobalMap(
    const std::shared_ptr<PoseGraph>& poseGraph) const
{
    const int nodeIdxMin = poseGraph->Nodes().front().Index();
    const int nodeIdxMax = poseGraph->Nodes().back().Index();

    /* Construct the global map */
    GridMapType gridMap { this->mMapResolution, this->mPatchSize,
                          0, 0, Point2D<double>(0.0, 0.0) };
    this->ConstructMapFromScans(gridMap, poseGraph, nodeIdxMin, nodeIdxMax);

    return gridMap;
}

/* Precompute grid maps for efficiency */
void GridMapBuilder::PrecomputeGridMaps(int localMapIdx, int nodeHeightMax)
{
    /* Retrieve the local grid map */
    auto& localMapInfo = this->mLocalMaps.at(localMapIdx);
    auto& precomputedMaps = localMapInfo.mPrecomputedMaps;
    const GridMapType& localMap = localMapInfo.mMap;

    /* The local grid map must be finished */
    assert(localMapInfo.mFinished);

    /* Create the temporary grid map to store the intermediate result */
    const double mapResolution = localMap.MapResolution();
    const Point2D<double>& minPos = localMap.MinPos();
    const Point2D<double> maxPos {
        minPos.mX + mapResolution * localMap.NumOfGridCellsX(),
        minPos.mY + mapResolution * localMap.NumOfGridCellsY() };
    PrecomputedMapType tmpMap {
        localMap.MapResolution(), localMap.PatchSize(),
        minPos.mX, minPos.mY, maxPos.mX, maxPos.mY };

    /* Make sure that the newly created grid map covers the local map */
    assert(tmpMap.MinPos() == localMap.MinPos());
    assert(tmpMap.NumOfPatchesX() >= localMap.NumOfPatchesX());
    assert(tmpMap.NumOfPatchesY() >= localMap.NumOfPatchesY());

    /* Compute a grid map for each node height */
    for (int nodeHeight = 0, winSize = 1;
         nodeHeight <= nodeHeightMax; ++nodeHeight, winSize *= 2) {
        /* Each pixel stores the maximum of the occupancy probability
         * values of the 2^h * 2^h box of pixels beginning there */
        /* Create a new grid map */
        PrecomputedMapType precompMap {
            localMap.MapResolution(), localMap.PatchSize(),
            minPos.mX, minPos.mY, maxPos.mX, maxPos.mY };

        /* Check the map size */
        assert(precompMap.MinPos() == tmpMap.MinPos());
        assert(precompMap.NumOfPatchesX() >= tmpMap.NumOfPatchesX());
        assert(precompMap.NumOfPatchesY() >= tmpMap.NumOfPatchesY());

        /* Store the maximum of the 2^h pixel wide row */
        this->SlidingWindowMaxRow(localMap, tmpMap, winSize);
        /* Store the maximum of the 2^h pixel wide column */
        this->SlidingWindowMaxCol(tmpMap, precompMap, winSize);

        /* Append the newly created grid map */
        precomputedMaps.emplace(nodeHeight, std::move(precompMap));
    }

    /* Mark the local map as precomputed */
    localMapInfo.mPrecomputed = true;
}

/* Update the grid map (list of the local grid maps) */
void GridMapBuilder::UpdateGridMap(
    const std::shared_ptr<PoseGraph>& poseGraph)
{
    /* Current robot pose (stored in the latest pose graph node) */
    const RobotPose2D<double>& robotPose = poseGraph->LatestNode().Pose();
    /* Current scan data (stored in the latest pose graph node) */
    const ScanType& scanData = poseGraph->LatestNode().ScanData();
    /* Current node index */
    const int nodeIdx = poseGraph->LatestNode().Index();

    /* Calculate the relative robot pose since the last method call */
    const RobotPose2D<double> relRobotPose = (this->mLocalMaps.size() == 0) ?
        RobotPose2D<double>(0.0, 0.0, 0.0) :
        InverseCompound(this->mLastRobotPose, robotPose);
    
    this->mLastRobotPose = robotPose;

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

        /* Create a new local map */
        const Point2D<double> centerPos { robotPose.mX, robotPose.mY };
        GridMapType newLocalMap {
            this->mMapResolution, this->mPatchSize, 0, 0, centerPos };
        this->mLocalMaps.emplace_back(std::move(newLocalMap), nodeIdx);

        /* Reset the variables properly */
        this->mTravelDistLastLocalMap = 0.0;
        this->mRobotPoseLastLocalMap = robotPose;
    }

    /* Local map to which the latest scan is added */
    LocalMapInfo& localMapInfo = this->mLocalMaps.back();
    GridMapType& localMap = localMapInfo.mMap;

    /* Compute the scan points and bounding box */
    Point2D<double> scanBottomLeft;
    Point2D<double> scanTopRight;
    std::vector<Point2D<double>> hitPoints;
    this->ComputeBoundingBoxAndScanPoints(
        robotPose, scanData, scanBottomLeft, scanTopRight, hitPoints);

    /* Expand the local map so that it can contain the latest scan */
    localMap.Expand(scanBottomLeft.mX, scanBottomLeft.mY,
                    scanTopRight.mX, scanTopRight.mY);
    
    /* Compute the sensor pose from the robot pose */
    const RobotPose2D<double> sensorPose =
        Compound(robotPose, scanData->RelativeSensorPose());
    /* Calculate the grid cell index corresponding to the sensor pose */
    const Point2D<int> sensorGridCellIdx =
        localMap.WorldCoordinateToGridCellIndex(sensorPose.mX, sensorPose.mY);

    /* Integrate the scan into the local map */
    const std::size_t numOfFilteredScans = hitPoints.size();

    for (std::size_t i = 0; i < numOfFilteredScans; ++i) {
        /* Compute the index of the hit grid cell */
        const Point2D<int> hitGridCellIdx =
            localMap.WorldCoordinateToGridCellIndex(hitPoints[i]);
        
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
    localMapInfo.mPoseGraphNodeIdxMax = nodeIdx;
}

/* Update the grid map with the latest scans */
void GridMapBuilder::UpdateLatestMap(
    const std::shared_ptr<PoseGraph>& poseGraph)
{
    /* Calculate the minimum and maximum node index */
    this->mLatestScanIdxMin = std::max(
        0, poseGraph->LatestNode().Index() - this->mNumOfScansForLatestMap + 1);
    this->mLatestScanIdxMax = poseGraph->LatestNode().Index();

    this->ConstructMapFromScans(
        this->mLatestMap, poseGraph,
        this->mLatestScanIdxMin, this->mLatestScanIdxMax);
}

/* Update the accumulated travel distance after the loop closure */
void GridMapBuilder::UpdateAccumTravelDist(
    const std::shared_ptr<PoseGraph>& poseGraph)
{
    this->mAccumTravelDist = 0.0;

    const int numOfNodes = static_cast<int>(poseGraph->Nodes().size());

    /* Accumulate the travel distance using pose graph nodes */
    for (int i = 1; i < numOfNodes; ++i) {
        const auto& prevNode = poseGraph->NodeAt(i - 1);
        const auto& node = poseGraph->NodeAt(i);

        this->mAccumTravelDist += Distance(prevNode.Pose(), node.Pose());
    }
}

/* Construct the grid map from the specified scans */
void GridMapBuilder::ConstructMapFromScans(
    GridMapType& gridMap,
    const std::shared_ptr<PoseGraph>& poseGraph,
    int poseGraphNodeIdxMin,
    int poseGraphNodeIdxMax) const
{
    /* Compute the scan points and bounding box */
    Point2D<double> bottomLeft { std::numeric_limits<double>::max(),
                                 std::numeric_limits<double>::max() };
    Point2D<double> topRight { std::numeric_limits<double>::min(),
                               std::numeric_limits<double>::min() };
    std::map<int, std::vector<Point2D<double>>> hitPoints;

    for (int nodeIdx = poseGraphNodeIdxMin;
         nodeIdx <= poseGraphNodeIdxMax; ++nodeIdx) {
        /* Retrieve the pose and scan data in the node */
        const PoseGraph::Node& node = poseGraph->NodeAt(nodeIdx);
        const RobotPose2D<double>& nodePose = node.Pose();
        const ScanType& scanData = node.ScanData();

        /* Compute the sensor pose from the node pose */
        const RobotPose2D<double> sensorPose =
            Compound(nodePose, scanData->RelativeSensorPose());
        
        bottomLeft.mX = std::min(bottomLeft.mX, sensorPose.mX);
        bottomLeft.mY = std::min(bottomLeft.mY, sensorPose.mY);
        topRight.mX = std::max(topRight.mX, sensorPose.mX);
        topRight.mY = std::max(topRight.mY, sensorPose.mY);

        /* Compute the minimum and maximum range of the scan */
        const double minRange = std::max(
            this->mUsableRangeMin, scanData->MinRange());
        const double maxRange = std::min(
            this->mUsableRangeMax, scanData->MaxRange());

        /* Compute the scan points and bounding box */
        const std::size_t numOfScans = scanData->NumOfScans();
        std::vector<Point2D<double>> nodeHitPoints;
        nodeHitPoints.reserve(numOfScans);

        for (std::size_t i = 0; i < numOfScans; ++i) {
            const double scanRange = scanData->RangeAt(i);

            if (scanRange >= maxRange || scanRange <= minRange)
                continue;
            
            /* Compute the hit grid cell point */
            const Point2D<double> hitPoint = scanData->HitPoint(sensorPose, i);
            nodeHitPoints.push_back(hitPoint);

            /* Update the bounding box */
            bottomLeft.mX = std::min(bottomLeft.mX, hitPoint.mX);
            bottomLeft.mY = std::min(bottomLeft.mY, hitPoint.mY);
            topRight.mX = std::max(topRight.mX, hitPoint.mX);
            topRight.mY = std::max(topRight.mY, hitPoint.mY);
        }

        hitPoints.emplace(nodeIdx, std::move(nodeHitPoints));
    }

    /* Create a new grid map that contains all scan points */
    gridMap.Resize(bottomLeft.mX, bottomLeft.mY,
                   topRight.mX, topRight.mY);
    gridMap.Reset();
    
    /* Integrate the scan into the grid map */
    for (int nodeIdx = poseGraphNodeIdxMin;
         nodeIdx <= poseGraphNodeIdxMax; ++nodeIdx) {
        /* Retrieve the pose in the node */
        const PoseGraph::Node& node = poseGraph->NodeAt(nodeIdx);
        const RobotPose2D<double>& nodePose = node.Pose();
        const ScanType& scanData = node.ScanData();

        /* Compute the sensor pose from the node pose */
        const RobotPose2D<double> sensorPose =
            Compound(nodePose, scanData->RelativeSensorPose());
        /* Compute the grid cell index corresponding to the sensor pose */
        const Point2D<int> sensorGridCellIdx =
            gridMap.WorldCoordinateToGridCellIndex(
                sensorPose.mX, sensorPose.mY);
        
        /* Integrate the scan into the grid map */
        const std::size_t numOfFilteredScans = hitPoints[nodeIdx].size();

        for (std::size_t i = 0; i < numOfFilteredScans; ++i) {
            /* Compute the index of the hit grid cell */
            const Point2D<int> hitGridCellIdx =
                gridMap.WorldCoordinateToGridCellIndex(
                    hitPoints[nodeIdx].at(i));
            
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

/* Compute the bounding box of the scan and scan points */
void GridMapBuilder::ComputeBoundingBoxAndScanPoints(
    const RobotPose2D<double>& robotPose,
    const ScanType& scanData,
    Point2D<double>& bottomLeft,
    Point2D<double>& topRight,
    std::vector<Point2D<double>>& hitPoints)
{
    /* Compute the sensor pose from the robot pose */
    const RobotPose2D<double> sensorPose =
        Compound(robotPose, scanData->RelativeSensorPose());
    
    /* Bottom-left corner position of the given scan */
    bottomLeft.mX = sensorPose.mX;
    bottomLeft.mY = sensorPose.mY;

    /* Top-right corner position of the given scan */
    topRight.mX = sensorPose.mX;
    topRight.mY = sensorPose.mY;

    const std::size_t numOfScans = scanData->NumOfScans();
    hitPoints.reserve(numOfScans);

    /* Minimum and maximum range of the scan */
    const double minRange = std::max(
        this->mUsableRangeMin, scanData->MinRange());
    const double maxRange = std::min(
        this->mUsableRangeMax, scanData->MaxRange());
    
    /* Calculate the bounding box and scan points */
    for (std::size_t i = 0; i < numOfScans; ++i) {
        const double scanRange = scanData->RangeAt(i);

        if (scanRange >= maxRange || scanRange <= minRange)
            continue;
        
        /* Calculate the hit grid cell point */
        const Point2D<double> hitPoint = scanData->HitPoint(sensorPose, i);
        hitPoints.push_back(hitPoint);

        /* Update the corner positions (bounding box) */
        bottomLeft.mX = std::min(bottomLeft.mX, hitPoint.mX);
        bottomLeft.mY = std::min(bottomLeft.mY, hitPoint.mY);
        topRight.mX = std::max(topRight.mX, hitPoint.mX);
        topRight.mY = std::max(topRight.mY, hitPoint.mY);
    }
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

/* Compute the maximum of a 2^h pixel wide row starting at each pixel */
void GridMapBuilder::SlidingWindowMaxRow(
    const GridMapType& gridMap,
    PrecomputedMapType& tmpMap,
    int winSize)
{
    /* Compute the maximum for each column */
    const double unknownVal = GridMapType::GridCellType::Unknown;
    int colIdx = 0;

    std::function<double(int)> inFunc =
        [&colIdx, &gridMap, unknownVal](int rowIdx) {
        return gridMap.Value(colIdx, rowIdx, unknownVal);
    };

    std::function<void(int, double)> outFunc =
        [&colIdx, &gridMap, &tmpMap, unknownVal](int rowIdx, double maxVal) {
        const Point2D<int> patchIdx =
            gridMap.GridCellIndexToPatchIndex(colIdx, rowIdx);
        const bool isAllocated = gridMap.PatchIsAllocated(patchIdx);

        if (isAllocated || maxVal != unknownVal)
            tmpMap.Update(colIdx, rowIdx, maxVal);
    };

    /* Apply the sliding window maximum function */
    for (colIdx = 0; colIdx < gridMap.NumOfGridCellsX(); ++colIdx)
        SlidingWindowMax(inFunc, outFunc, gridMap.NumOfGridCellsY(), winSize);
}

/* Compute the maximum of a 2^h pixel wide column starting at each pixel */
void GridMapBuilder::SlidingWindowMaxCol(
    const PrecomputedMapType& tmpMap,
    PrecomputedMapType& precompMap,
    int winSize)
{
    /* Compute the maximum for each row */
    const double unknownVal = GridMapType::GridCellType::Unknown;
    int rowIdx = 0;

    std::function<double(int)> inFunc =
        [&rowIdx, &tmpMap, unknownVal](int colIdx) {
        return tmpMap.Value(colIdx, rowIdx, unknownVal);
    };

    std::function<void(int, double)> outFunc =
        [&rowIdx, &tmpMap, &precompMap, unknownVal](int colIdx, double maxVal) {
        const Point2D<int> patchIdx =
            tmpMap.GridCellIndexToPatchIndex(colIdx, rowIdx);
        const bool isAllocated = tmpMap.PatchIsAllocated(patchIdx);

        if (isAllocated || maxVal != unknownVal)
            precompMap.Update(colIdx, rowIdx, maxVal);
    };

    /* Apply the sliding window maximum function */
    for (rowIdx = 0; rowIdx < tmpMap.NumOfGridCellsY(); ++rowIdx)
        SlidingWindowMax(inFunc, outFunc, tmpMap.NumOfGridCellsX(), winSize);
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
