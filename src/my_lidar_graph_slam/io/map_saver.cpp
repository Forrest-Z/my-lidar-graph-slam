
/* map_saver.cpp */

#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "my_lidar_graph_slam/point.hpp"
#include "my_lidar_graph_slam/pose.hpp"
#include "my_lidar_graph_slam/grid_map/grid_map.hpp"
#include "my_lidar_graph_slam/io/map_saver.hpp"

/* Declare namespaces for convenience */
namespace gil = boost::gil;
namespace pt = boost::property_tree;

namespace MyLidarGraphSlam {
namespace IO {

/* Get the MapSaver instance */
MapSaver* MapSaver::Instance()
{
    static MapSaver theInstance;
    return &theInstance;
}

/* Save the entire map */
bool MapSaver::SaveMap(const GridMapBuilderPtr& gridMapBuilder,
                       const PoseGraphPtr& poseGraph,
                       const std::string& fileName,
                       bool drawTrajectory,
                       bool saveMetadata) const
{
    assert(gridMapBuilder->LocalMaps().size() > 0 &&
           "Grid map is empty (contains no local map)");
    assert(poseGraph->Nodes().size() > 0 &&
           "Pose graph is empty (contains no node)");

    Options saveOptions;
    saveOptions.mDrawTrajectory = drawTrajectory;
    saveOptions.mTrajectoryNodeIdxMin = 0;
    saveOptions.mTrajectoryNodeIdxMax = poseGraph->Nodes().size() - 1;
    saveOptions.mDrawScans = false;
    saveOptions.mSaveMetadata = saveMetadata;
    saveOptions.mFileName = fileName;

    /* Construct the map that accommodates all scan data */
    const GridMapType globalMap =
        gridMapBuilder->ConstructGlobalMap(poseGraph);
    
    /* Save the map image and metadata */
    return this->SaveMapCore(globalMap, poseGraph, saveOptions);
}

/* Save the pose graph as JSON format */
bool MapSaver::SavePoseGraph(const PoseGraphPtr& poseGraph,
                             const std::string& fileName) const
{
    pt::ptree jsonPoseGraph;

    /* Write the pose graph nodes */
    pt::ptree poseGraphNodes;

    for (const auto& node : poseGraph->Nodes()) {
        pt::ptree nodeInfo;
        nodeInfo.put("Index", node.Index());
        nodeInfo.put("Pose.X", node.Pose().mX);
        nodeInfo.put("Pose.Y", node.Pose().mY);
        nodeInfo.put("Pose.Theta", node.Pose().mTheta);
        nodeInfo.put("TimeStamp", node.ScanData()->TimeStamp());
        poseGraphNodes.push_back(std::make_pair("", nodeInfo));
    }

    jsonPoseGraph.add_child("PoseGraph.Nodes", poseGraphNodes);

    /* Write the pose graph edges */
    pt::ptree poseGraphEdges;

    for (const auto& edge : poseGraph->Edges()) {
        pt::ptree edgeInfo;
        edgeInfo.put("StartNodeIdx", edge.StartNodeIndex());
        edgeInfo.put("EndNodeIdx", edge.EndNodeIndex());
        edgeInfo.put("RelativePose.X", edge.RelativePose().mX);
        edgeInfo.put("RelativePose.Y", edge.RelativePose().mY);
        edgeInfo.put("RelativePose.Theta", edge.RelativePose().mTheta);
        
        /* Store information matrix elements (upper triangular) */
        pt::ptree infoMatElements;
        const Eigen::Matrix3d& infoMat = edge.InformationMatrix();

        for (Eigen::Index i = 0; i < infoMat.rows(); ++i) {
            for (Eigen::Index j = i; j < infoMat.cols(); ++j) {
                pt::ptree matrixElement;
                matrixElement.put_value(infoMat(i, j));
                infoMatElements.push_back(std::make_pair("", matrixElement));
            }
        }

        edgeInfo.add_child("InformationMatrix", infoMatElements);
        poseGraphEdges.push_back(std::make_pair("", edgeInfo));
    }

    jsonPoseGraph.add_child("PoseGraph.Edges", poseGraphEdges);

    /* Save the pose graph as JSON format */
    try {
        const std::string poseGraphFileName = fileName + ".posegraph.json";
        pt::write_json(poseGraphFileName, jsonPoseGraph);
    } catch (const pt::json_parser_error& e) {
        std::cerr << "boost::property_tree::json_parser_error occurred: "
                  << e.what()
                  << "(" << e.filename()
                  << ", Line " << e.line() << ")" << std::endl;
        return false;
    }

    return true;
}

/* Save local maps individually */
bool MapSaver::SaveLocalMaps(const GridMapBuilderPtr& gridMapBuilder,
                             const PoseGraphPtr& poseGraph,
                             const std::string& fileName,
                             bool drawTrajectory,
                             bool saveMetadata) const
{
    assert(gridMapBuilder->LocalMaps().size() > 0 &&
           "Grid map is empty (contains no local map)");
    assert(poseGraph->Nodes().size() > 0 &&
           "Pose graph is empty (contains no node)");
    
    /* Save local maps individually as PNG images */
    for (std::size_t i = 0; i < gridMapBuilder->LocalMaps().size(); ++i) {
        /* Retrieve the local map */
        const auto& localMapInfo = gridMapBuilder->LocalMapAt(i);
        const std::string localMapFileName =
            fileName + "-localmap-" + std::to_string(i);
        
        Options saveOptions;
        saveOptions.mDrawTrajectory = drawTrajectory;
        saveOptions.mTrajectoryNodeIdxMin = localMapInfo.mPoseGraphNodeIdxMin;
        saveOptions.mTrajectoryNodeIdxMax = localMapInfo.mPoseGraphNodeIdxMax;
        saveOptions.mDrawScans = false;
        saveOptions.mSaveMetadata = saveMetadata;
        saveOptions.mFileName = localMapFileName;

        /* Save the map image and metadata */
        if (!this->SaveMapCore(localMapInfo.mMap, poseGraph, saveOptions))
            return false;
    }

    return true;
}

/* Save the grid map constructed from the latest scans */
bool MapSaver::SaveLatestMap(const GridMapBuilderPtr& gridMapBuilder,
                             const PoseGraphPtr& poseGraph,
                             const std::string& fileName,
                             bool drawTrajectory,
                             bool saveMetadata) const
{
    const std::string latestMapFileName = fileName + "-latest-scans";

    Options saveOptions;
    saveOptions.mDrawTrajectory = drawTrajectory;
    saveOptions.mTrajectoryNodeIdxMin = gridMapBuilder->LatestScanIdxMin();
    saveOptions.mTrajectoryNodeIdxMax = gridMapBuilder->LatestScanIdxMax();
    saveOptions.mDrawScans = false;
    saveOptions.mSaveMetadata = saveMetadata;
    saveOptions.mFileName = latestMapFileName;

    /* Save the map image and metadata */
    return this->SaveMapCore(
        gridMapBuilder->LatestMap(), poseGraph, saveOptions);
}

/* Save the map and the scan */
bool MapSaver::SaveLocalMapAndScan(const GridMapBuilderPtr& gridMapBuilder,
                                   const PoseGraphPtr& poseGraph,
                                   const std::size_t localMapIdx,
                                   const RobotPose2D<double>& scanPose,
                                   const ScanPtr& scanData,
                                   bool drawTrajectory,
                                   bool saveMetadata,
                                   const std::string& fileName) const
{
    const auto& localMapInfo = gridMapBuilder->LocalMapAt(localMapIdx);

    Options saveOptions;
    saveOptions.mDrawTrajectory = drawTrajectory;
    saveOptions.mTrajectoryNodeIdxMin = localMapInfo.mPoseGraphNodeIdxMin;
    saveOptions.mTrajectoryNodeIdxMax = localMapInfo.mPoseGraphNodeIdxMax;
    saveOptions.mDrawScans = true;
    saveOptions.mScanPose = scanPose;
    saveOptions.mScanData = scanData;
    saveOptions.mSaveMetadata = saveMetadata;
    saveOptions.mFileName = fileName;
    
    /* Save the map image */
    return this->SaveMapCore(localMapInfo.mMap, poseGraph, saveOptions);
}

/* Save the latest map and the scan */
bool MapSaver::SaveLatestMapAndScan(const GridMapBuilderPtr& gridMapBuilder,
                                    const PoseGraphPtr& poseGraph,
                                    const RobotPose2D<double>& scanPose,
                                    const ScanPtr& scanData,
                                    bool drawTrajectory,
                                    bool saveMetadata,
                                    const std::string& fileName) const
{
    Options saveOptions;
    saveOptions.mDrawTrajectory = drawTrajectory;
    saveOptions.mTrajectoryNodeIdxMin = gridMapBuilder->LatestScanIdxMin();
    saveOptions.mTrajectoryNodeIdxMax = gridMapBuilder->LatestScanIdxMax();
    saveOptions.mDrawScans = true;
    saveOptions.mScanPose = scanPose;
    saveOptions.mScanData = scanData;
    saveOptions.mSaveMetadata = saveMetadata;
    saveOptions.mFileName = fileName;

    /* Save the map image */
    return this->SaveMapCore(
        gridMapBuilder->LatestMap(), poseGraph, saveOptions);
}

/* Save the precomputed grid maps */
bool MapSaver::SavePrecomputedGridMaps(const GridMapBuilderPtr& gridMapBuilder,
                                       const PoseGraphPtr& poseGraph,
                                       const int mapIdx,
                                       const std::string& fileName) const
{
    Options saveOptions;
    saveOptions.mDrawTrajectory = false;
    saveOptions.mDrawScans = false;
    saveOptions.mSaveMetadata = false;

    const auto& localMapInfo = gridMapBuilder->LocalMapAt(mapIdx);
    const auto& precompMaps = localMapInfo.mPrecomputedMaps;

    /* Convert the precomputed grid map and save */
    for (const auto& precompMapInfo : precompMaps) {
        const int nodeHeight = precompMapInfo.first;
        const auto& precompMap = precompMapInfo.second;

        /* Create the occupancy grid map */
        GridMapType gridMap = GridMapType::CreateSameSizeMap(precompMap);

        const double unknownVal = PrecomputedMapType::GridCellType::Unknown;
        const int numOfGridCellsX = precompMap.NumOfGridCellsX();
        const int numOfGridCellsY = precompMap.NumOfGridCellsY();

        /* Copy the occupancy probability values */
        for (int y = 0; y < numOfGridCellsY; ++y) {
            for (int x = 0; x < numOfGridCellsX; ++x) {
                const double mapValue = precompMap.Value(x, y, unknownVal);

                if (mapValue != unknownVal)
                    gridMap.Update(x, y, mapValue);
            }
        }

        /* Save the map image */
        const int winSize = 1 << nodeHeight;
        saveOptions.mFileName = fileName + "-" + std::to_string(winSize);

        if (!this->SaveMapCore(gridMap, poseGraph, saveOptions))
            return false;
    }

    return true;
}

/* Determine the actual map size */
void MapSaver::ComputeActualMapSize(const GridMapType& gridMap,
                                    Point2D<int>& patchIdxMin,
                                    Point2D<int>& gridCellIdxMin,
                                    Point2D<int>& mapSizeInPatches,
                                    Point2D<int>& mapSizeInGridCells) const
{
    /* Get the range of the patch index (bounding box)
     * [patchIdxMin.mX, patchIdxMax.mX], [patchIdxMin.mY, patchIdxMax.mY] */
    patchIdxMin.mX = std::numeric_limits<int>::max();
    patchIdxMin.mY = std::numeric_limits<int>::max();
    
    Point2D<int> patchIdxMax;
    patchIdxMax.mX = std::numeric_limits<int>::min();
    patchIdxMax.mY = std::numeric_limits<int>::min();
    
    for (int y = 0; y < gridMap.NumOfPatchesY(); ++y) {
        for (int x = 0; x < gridMap.NumOfPatchesX(); ++x) {
            if (!gridMap.PatchIsAllocated(x, y))
                continue;
            
            patchIdxMin.mX = std::min(patchIdxMin.mX, x);
            patchIdxMin.mY = std::min(patchIdxMin.mY, y);
            patchIdxMax.mX = std::max(patchIdxMax.mX, x);
            patchIdxMax.mY = std::max(patchIdxMax.mY, y);
        }
    }

    /* Convert the patch index ranges to grid cell index ranges */
    Point2D<int> gridCellIdxTmp;
    Point2D<int> gridCellIdxMax;
    gridMap.PatchIndexToGridCellIndexRange(
        patchIdxMin, gridCellIdxMin, gridCellIdxTmp);
    gridMap.PatchIndexToGridCellIndexRange(
        patchIdxMax, gridCellIdxTmp, gridCellIdxMax);
    
    /* Compute the actual map size */
    mapSizeInPatches.mX = patchIdxMax.mX - patchIdxMin.mX + 1;
    mapSizeInPatches.mY = patchIdxMax.mY - patchIdxMin.mY + 1;
    mapSizeInGridCells.mX = gridCellIdxMax.mX - gridCellIdxMin.mX;
    mapSizeInGridCells.mY = gridCellIdxMax.mY - gridCellIdxMin.mY;

    assert(mapSizeInGridCells.mX ==
           mapSizeInPatches.mX * gridMap.PatchSize());
    assert(mapSizeInGridCells.mY ==
           mapSizeInPatches.mY * gridMap.PatchSize());
}

/* Draw the grid cells to the image */
void MapSaver::DrawMap(const GridMapType& gridMap,
                       const gil::rgb8_view_t& mapImageView,
                       const Point2D<int>& patchIdxMin,
                       const Point2D<int>& mapSizeInPatches) const
{
    /* Draw the patches to the image */
    for (int y = 0; y < mapSizeInPatches.mY; ++y) {
        for (int x = 0; x < mapSizeInPatches.mX; ++x) {
            const Mapping::GridMapBuilder::PatchType& patch =
                gridMap.PatchAt(patchIdxMin.mX + x, patchIdxMin.mY + y);
            
            if (!patch.IsAllocated())
                continue;
            
            /* Draw the grid cells in the patch */
            for (int yy = 0; yy < gridMap.PatchSize(); ++yy) {
                for (int xx = 0; xx < gridMap.PatchSize(); ++xx) {
                    const double gridCellValue = patch.At(xx, yy).Value();

                    /* If the occupancy probability value is less than or
                     * equal to zero, then the grid cell is not yet observed
                     * and is in unknown state (GridCell::Unknown is zero) */
                    if (gridCellValue <= 0.0 || gridCellValue > 1.0)
                        continue;

                    const std::ptrdiff_t idxX = static_cast<std::ptrdiff_t>(
                        x * gridMap.PatchSize() + xx);
                    const std::ptrdiff_t idxY = static_cast<std::ptrdiff_t>(
                        y * gridMap.PatchSize() + yy);
                    const std::uint8_t grayScale = static_cast<std::uint8_t>(
                        (1.0 - gridCellValue) * 255.0);

                    mapImageView(idxX, idxY) =
                        gil::rgb8_pixel_t(grayScale, grayScale, grayScale);
                }
            }
        }
    }
}

/* Draw the trajectory lines to the image */
void MapSaver::DrawTrajectory(const GridMapType& gridMap,
                              const PoseGraphPtr& poseGraph,
                              const gil::rgb8_view_t& mapImageView,
                              const Point2D<int>& gridCellIdxMin,
                              const Point2D<int>& mapSizeInGridCells,
                              const std::size_t nodeIdxMin,
                              const std::size_t nodeIdxMax) const
{
    /* Input validity checks */
    assert(nodeIdxMin < poseGraph->Nodes().size());
    assert(nodeIdxMax < poseGraph->Nodes().size());

    /* Draw the trajectory lines to the image */
    const auto& initialNode = poseGraph->NodeAt(nodeIdxMin);
    Point2D<int> prevGridCellIdx =
        gridMap.WorldCoordinateToGridCellIndex(
            initialNode.Pose().mX, initialNode.Pose().mY);
    
    const Point2D<int> gridCellIdxMax {
        gridCellIdxMin.mX + mapSizeInGridCells.mX,
        gridCellIdxMin.mY + mapSizeInGridCells.mY };

    for (std::size_t i = nodeIdxMin + 1; i <= nodeIdxMax; ++i) {
        const auto& node = poseGraph->NodeAt(i);
        const Point2D<int> gridCellIdx =
            gridMap.WorldCoordinateToGridCellIndex(
                node.Pose().mX, node.Pose().mY);
        const std::vector<Point2D<int>> lineIndices =
            Bresenham(prevGridCellIdx, gridCellIdx);
        
        for (const Point2D<int>& interpolatedIdx : lineIndices) {
            if (interpolatedIdx.mX < gridCellIdxMin.mX ||
                interpolatedIdx.mX >= gridCellIdxMax.mX - 1 ||
                interpolatedIdx.mY < gridCellIdxMin.mY ||
                interpolatedIdx.mY >= gridCellIdxMax.mY - 1)
                continue;
            
            const int x = interpolatedIdx.mX - gridCellIdxMin.mX;
            const int y = interpolatedIdx.mY - gridCellIdxMin.mY;
            gil::fill_pixels(gil::subimage_view(mapImageView, x, y, 2, 2),
                             gil::rgb8_pixel_t(255, 0, 0));
        }

        prevGridCellIdx = gridCellIdx;
    }
}

/* Draw the scans obtained at the specified node to the image */
void MapSaver::DrawScan(const GridMapType& gridMap,
                        const boost::gil::rgb8_view_t& mapImageView,
                        const Point2D<int>& gridCellIdxMin,
                        const Point2D<int>& mapSizeInGridCells,
                        const RobotPose2D<double>& scanPose,
                        const ScanPtr& scanData) const
{
    const Point2D<int> gridCellIdxMax {
        gridCellIdxMin.mX + mapSizeInGridCells.mX,
        gridCellIdxMin.mY + mapSizeInGridCells.mY };
    
    /* Draw the pose where the scan data is obtained to the image */
    const Point2D<int> scanPoseIdx =
        gridMap.WorldCoordinateToGridCellIndex(scanPose.mX, scanPose.mY);
    
    if (scanPoseIdx.mX >= gridCellIdxMin.mX &&
        scanPoseIdx.mX < gridCellIdxMax.mX - 2 &&
        scanPoseIdx.mY >= gridCellIdxMin.mY &&
        scanPoseIdx.mY < gridCellIdxMax.mX - 2) {
        const int x = scanPoseIdx.mX - gridCellIdxMin.mX;
        const int y = scanPoseIdx.mY - gridCellIdxMin.mY;
        gil::fill_pixels(gil::subimage_view(mapImageView, x, y, 3, 3),
                         gil::rgb8_pixel_t(0, 255, 0));
    }

    const RobotPose2D<double> sensorPose =
        Compound(scanPose, scanData->RelativeSensorPose());
    const std::size_t numOfScans = scanData->NumOfScans();

    for (std::size_t i = 0; i < numOfScans; ++i) {
        /* Calculate the grid cell index */
        const Point2D<double> hitPoint =
            scanData->HitPoint(sensorPose, i);
        const Point2D<int> hitPointIdx =
            gridMap.WorldCoordinateToGridCellIndex(hitPoint);
        
        if (hitPointIdx.mX < gridCellIdxMin.mX ||
            hitPointIdx.mX >= gridCellIdxMax.mX - 1 ||
            hitPointIdx.mY < gridCellIdxMin.mY ||
            hitPointIdx.mY >= gridCellIdxMax.mY - 1)
            continue;

        /* Draw the scan point to the image */
        const int x = hitPointIdx.mX - gridCellIdxMin.mX;
        const int y = hitPointIdx.mY - gridCellIdxMin.mY;
        gil::fill_pixels(gil::subimage_view(mapImageView, x, y, 2, 2),
                         gil::rgb8_pixel_t(0, 0, 255));
    }
}

/* Save the map image and metadata */
bool MapSaver::SaveMapCore(const GridMapType& gridMap,
                           const PoseGraphPtr& poseGraph,
                           const Options& saveOptions) const
{
    /* Compute the map size to be written to the image */
    Point2D<int> patchIdxMin;
    Point2D<int> gridCellIdxMin;
    Point2D<int> mapSizeInPatches;
    Point2D<int> mapSizeInGridCells;
    this->ComputeActualMapSize(gridMap,
                               patchIdxMin, gridCellIdxMin,
                               mapSizeInPatches, mapSizeInGridCells);

    /* Initialize the image */
    gil::rgb8_image_t mapImage { mapSizeInGridCells.mX,
                                 mapSizeInGridCells.mY };
    const gil::rgb8_view_t& mapImageView = gil::view(mapImage);
    gil::fill_pixels(mapImageView, gil::rgb8_pixel_t(192, 192, 192));
    
    /* Draw the grid cells to the image */
    this->DrawMap(gridMap, mapImageView,
                  patchIdxMin, mapSizeInPatches);

    /* Draw the trajectory (pose graph nodes) of the robot */
    if (saveOptions.mDrawTrajectory)
        this->DrawTrajectory(gridMap, poseGraph, mapImageView,
                             gridCellIdxMin, mapSizeInGridCells,
                             saveOptions.mTrajectoryNodeIdxMin,
                             saveOptions.mTrajectoryNodeIdxMax);
    
    /* Draw the scans to the image */
    if (saveOptions.mDrawScans)
        this->DrawScan(gridMap, mapImageView,
                       gridCellIdxMin, mapSizeInGridCells,
                       saveOptions.mScanPose, saveOptions.mScanData);

    /* Save the map as PNG image
     * Image should be flipped upside down */
    try {
        const std::string pngFileName = saveOptions.mFileName + ".png";
#if BOOST_VERSION <= 106700
        gil::png_write_view(pngFileName,
                            gil::flipped_up_down_view(mapImageView));
#else
        gil::write_view(pngFileName,
                        gil::flipped_up_down_view(mapImageView),
                        gil::png_tag());
#endif
    } catch (const std::ios_base::failure& e) {
        std::cerr << "std::ios_base::failure occurred: " << e.what() << ' '
                  << "(Error code: " << e.code() << ")" << std::endl;
        return false;
    }

    if (!saveOptions.mSaveMetadata)
        return true;
    
    /* Save the map metadata as JSON format */
    try {
        const std::string metadataFileName = saveOptions.mFileName + ".json";
        const Point2D<double> bottomLeft =
            gridMap.GridCellIndexToWorldCoordinate(gridCellIdxMin);
        const Point2D<double> topRight {
            bottomLeft.mX + mapSizeInGridCells.mX * gridMap.Resolution(),
            bottomLeft.mY + mapSizeInGridCells.mY * gridMap.Resolution() };
        this->SaveMapMetadata(
            gridMap.Resolution(), gridMap.PatchSize(),
            mapSizeInPatches, mapSizeInGridCells,
            bottomLeft, topRight,
            saveOptions.mTrajectoryNodeIdxMin,
            saveOptions.mTrajectoryNodeIdxMax,
            metadataFileName);
    } catch (const pt::json_parser_error& e) {
        std::cerr << "boost::property_tree::json_parser_error occurred: "
                  << e.what() << ' '
                  << "(" << e.filename() << ", Line " << e.line() << ")"
                  << std::endl;
        return false;
    }

    return true;
}

/* Save the map metadata as JSON format */
void MapSaver::SaveMapMetadata(double mapResolution,
                               int patchSize,
                               const Point2D<int>& mapSizeInPatches,
                               const Point2D<int>& mapSizeInGridCells,
                               const Point2D<double>& bottomLeft,
                               const Point2D<double>& topRight,
                               const std::size_t nodeIdxMin,
                               const std::size_t nodeIdxMax,
                               const std::string& fileName) const
{
    pt::ptree jsonMetadata;

    /* Write the map metadata */
    jsonMetadata.put("Map.Resolution", mapResolution);
    jsonMetadata.put("Map.PatchSize", patchSize);
    jsonMetadata.put("Map.WidthInPatches", mapSizeInPatches.mX);
    jsonMetadata.put("Map.HeightInPatches", mapSizeInPatches.mY);
    jsonMetadata.put("Map.WidthInGridCells", mapSizeInGridCells.mX);
    jsonMetadata.put("Map.HeightInGridCells", mapSizeInGridCells.mY);

    jsonMetadata.put("Map.BottomLeft.X", bottomLeft.mX);
    jsonMetadata.put("Map.BottomLeft.Y", bottomLeft.mY);
    jsonMetadata.put("Map.TopRight.X", topRight.mX);
    jsonMetadata.put("Map.TopRight.Y", topRight.mY);

    jsonMetadata.put("Map.PoseGraphNodeIdxMin", nodeIdxMin);
    jsonMetadata.put("Map.PoseGraphNodeIdxMax", nodeIdxMax);

    /* Save the map metadata as JSON format */
    pt::write_json(fileName, jsonMetadata);

    return;
}

} /* namespace IO */
} /* namespace MyLidarGraphSlam */
