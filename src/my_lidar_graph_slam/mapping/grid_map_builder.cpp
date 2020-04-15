
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
    mPoseGraphNodeIdxMax(other.mPoseGraphNodeIdxMax)
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
    double usableRangeMax) :
    mMapResolution(mapResolution),
    mPatchSize(patchSize),
    mLatestMap(mapResolution, patchSize, 0, 0, Point2D<double>(0.0, 0.0)),
    mNumOfScansForLatestMap(numOfScansForLatestMap),
    mLatestScanIdxMin(0),
    mLatestScanIdxMax(0),
    mRobotPoseLastLocalMap(0.0, 0.0, 0.0),
    mTravelDistThreshold(travelDistThreshold),
    mUsableRangeMin(usableRangeMin),
    mUsableRangeMax(usableRangeMax)
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

/* Construct the global grid map */
GridMapBuilder::GridMapType GridMapBuilder::ConstructGlobalMap(
    const std::shared_ptr<PoseGraph>& poseGraph) const
{
    const int nodeIdxMin = poseGraph->Nodes().front().Index();
    const int nodeIdxMax = poseGraph->Nodes().back().Index();

    return this->ConstructMapFromScans(poseGraph, nodeIdxMin, nodeIdxMax);
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

    /* Compute the sensor pose from the robot pose
     * Do not calculate the grid cell index corresponding to the sensor pose
     * here because the map parameters are updated later */
    const RobotPose2D<double> sensorPose =
        Compound(robotPose, scanData->RelativeSensorPose());

    /* Update the accumulated travel distance since the last grid map */
    const double accumulatedTravelDist = std::sqrt(
        std::pow(robotPose.mX - this->mRobotPoseLastLocalMap.mX, 2.0) +
        std::pow(robotPose.mY - this->mRobotPoseLastLocalMap.mY, 2.0));
    
    /* Determine whether to create a new local map */
    const bool travelDistThreshold =
        accumulatedTravelDist >= this->mTravelDistThreshold;
    const bool isFirstScan = this->mLocalMaps.size() == 0;
    const bool createNewLocalMap = travelDistThreshold || isFirstScan;

    /* Create a new local map if necessary */
    if (createNewLocalMap) {
        const Point2D<double> centerPos { sensorPose.mX, sensorPose.mY };
        GridMapType newLocalMap {
            this->mMapResolution, this->mPatchSize, 0, 0, centerPos };
        this->mLocalMaps.emplace_back(std::move(newLocalMap), nodeIdx);
        this->mRobotPoseLastLocalMap = robotPose;
    }

    /* Local map to which the latest scan is added */
    LocalMapInfo& localMapInfo = this->mLocalMaps.back();

    /* Compute the scan points and bounding box */
    Point2D<double> scanBottomLeft;
    Point2D<double> scanTopRight;
    std::vector<Point2D<double>> hitPoints;
    this->ComputeBoundingBoxAndScanPoints(
        robotPose, scanData, scanBottomLeft, scanTopRight, hitPoints);

    /* Expand the local map so that it can contain the latest scan */
    localMapInfo.mMap.Expand(scanBottomLeft.mX, scanBottomLeft.mY,
                             scanTopRight.mX, scanTopRight.mY);
    
    /* Calculate the grid cell index corresponding to the sensor pose here
     * since the map parameters are updated as above */
    const Point2D<int> sensorGridCellIdx =
        localMapInfo.mMap.WorldCoordinateToGridCellIndex(
            sensorPose.mX, sensorPose.mY);

    /* Integrate the scan into the local map */
    const std::size_t numOfFilteredScans = hitPoints.size();

    for (std::size_t i = 0; i < numOfFilteredScans; ++i) {
        /* Compute the index of the hit grid cell */
        const Point2D<int> hitGridCellIdx =
            localMapInfo.mMap.WorldCoordinateToGridCellIndex(hitPoints[i]);
        
        /* Compute the indices of the missed grid cells */
        const std::vector<Point2D<int>> missedGridCellIndices =
            this->ComputeMissedGridCellIndices(
                sensorGridCellIdx, hitGridCellIdx);
        
        /* Update occupancy probability values for missed grid cells */
        for (std::size_t j = 0; j < missedGridCellIndices.size(); ++j)
            localMapInfo.mMap.Update(missedGridCellIndices[j], false);
        
        /* Update occupancy probability values for hit grid cell */
        localMapInfo.mMap.Update(hitGridCellIdx, true);
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

    this->mLatestMap = this->ConstructMapFromScans(
        poseGraph, this->mLatestScanIdxMin, this->mLatestScanIdxMax);
}

/* Construct the grid map from the specified scans */
GridMapBuilder::GridMapType GridMapBuilder::ConstructMapFromScans(
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
        const std::size_t numOfScans = scanData->Ranges().size();
        std::vector<Point2D<double>> nodeHitPoints;
        nodeHitPoints.reserve(numOfScans);

        for (std::size_t i = 0; i < numOfScans; ++i) {
            const double range = scanData->Ranges().at(i);
            const double angle = scanData->MinAngle() +
                static_cast<double>(i) * scanData->AngleIncrement();
            const double cosTheta = std::cos(sensorPose.mTheta + angle);
            const double sinTheta = std::sin(sensorPose.mTheta + angle);

            if (range >= maxRange || range <= minRange)
                continue;
            
            /* Compute the hit grid cell point */
            nodeHitPoints.emplace_back(sensorPose.mX + range * cosTheta,
                                       sensorPose.mY + range * sinTheta);
            const Point2D<double>& hitPoint = nodeHitPoints.back();

            /* Update the bounding box */
            bottomLeft.mX = std::min(bottomLeft.mX, hitPoint.mX);
            bottomLeft.mY = std::min(bottomLeft.mY, hitPoint.mY);
            topRight.mX = std::max(topRight.mX, hitPoint.mX);
            topRight.mY = std::max(topRight.mY, hitPoint.mY);
        }

        hitPoints.emplace(nodeIdx, std::move(nodeHitPoints));
    }

    /* Create a new grid map that contains all scan points */
    GridMapType gridMap { this->mMapResolution, this->mPatchSize,
                          bottomLeft.mX, bottomLeft.mY,
                          topRight.mX, topRight.mY };
    
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
                gridMap.Update(missedGridCellIndices[j], false);
            
            /* Update occupancy probability values for hit grid cell */
            gridMap.Update(hitGridCellIdx, true);
        }
    }

    return gridMap;
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

    const std::size_t numOfScans = scanData->Ranges().size();
    hitPoints.reserve(numOfScans);

    /* Minimum and maximum range of the scan */
    const double minRange = std::max(
        this->mUsableRangeMin, scanData->MinRange());
    const double maxRange = std::min(
        this->mUsableRangeMax, scanData->MaxRange());
    
    /* Calculate the bounding box and scan points */
    for (std::size_t i = 0; i < numOfScans; ++i) {
        const double range = scanData->Ranges().at(i);
        const double angle = scanData->MinAngle() +
            static_cast<double>(i) * scanData->AngleIncrement();
        const double cosTheta = std::cos(sensorPose.mTheta + angle);
        const double sinTheta = std::sin(sensorPose.mTheta + angle);

        if (range >= maxRange || range <= minRange)
            continue;
        
        /* Calculate the hit grid cell point */
        hitPoints.emplace_back(sensorPose.mX + range * cosTheta,
                               sensorPose.mY + range * sinTheta);
        const Point2D<double>& hitPoint = hitPoints.back();

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

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
