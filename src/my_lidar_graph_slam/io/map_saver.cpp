
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
bool MapSaver::SaveMap(
    const Mapping::GridMapType& globalMap,
    const std::vector<Mapping::PoseGraph::Node>& poseGraphNodes,
    const std::string& fileName,
    const bool drawTrajectory,
    const bool saveMetadata) const
{
    assert(!poseGraphNodes.empty() &&
           "Pose graph is empty (contains no node)");

    Options saveOptions;
    saveOptions.mDrawTrajectory = drawTrajectory;
    saveOptions.mTrajectoryNodeIdxMin = 0;
    saveOptions.mTrajectoryNodeIdxMax = poseGraphNodes.size() - 1;
    saveOptions.mDrawScans = false;
    saveOptions.mSaveMetadata = saveMetadata;
    saveOptions.mFileName = fileName;

    /* Global map accommodates all local grid maps acquired */
    /* Save the map image and metadata */
    return this->SaveMapCore(globalMap, poseGraphNodes, saveOptions);
}

/* Save the pose graph as JSON format */
bool MapSaver::SavePoseGraph(
    const std::vector<Mapping::PoseGraph::Node>& poseGraphNodes,
    const std::vector<Mapping::PoseGraph::Edge>& poseGraphEdges,
    const std::string& fileName) const
{
    pt::ptree jsonPoseGraph;

    /* Write the pose graph nodes */
    pt::ptree poseGraphNodesTree;

    for (const auto& node : poseGraphNodes) {
        pt::ptree nodeInfo;
        nodeInfo.put("Index", node.Index());
        nodeInfo.put("Pose.X", node.Pose().mX);
        nodeInfo.put("Pose.Y", node.Pose().mY);
        nodeInfo.put("Pose.Theta", node.Pose().mTheta);
        nodeInfo.put("TimeStamp", node.ScanData()->TimeStamp());
        poseGraphNodesTree.push_back(std::make_pair("", nodeInfo));
    }

    jsonPoseGraph.add_child("PoseGraph.Nodes", poseGraphNodesTree);

    /* Write the pose graph edges */
    pt::ptree poseGraphEdgesTree;

    for (const auto& edge : poseGraphEdges) {
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
        poseGraphEdgesTree.push_back(std::make_pair("", edgeInfo));
    }

    jsonPoseGraph.add_child("PoseGraph.Edges", poseGraphEdgesTree);

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
bool MapSaver::SaveLocalMaps(
    const std::vector<Mapping::LocalMapInfo>& localMaps,
    const std::vector<Mapping::PoseGraph::Node>& poseGraphNodes,
    const bool drawTrajectory,
    const bool saveMetadata,
    const std::string& fileName) const
{
    assert(!localMaps.empty() &&
           "Grid map is empty (contains no local map)");
    assert(!poseGraphNodes.empty() &&
           "Pose graph is empty (contains no node)");

    /* Save local maps individually as PNG images */
    for (std::size_t i = 0; i < localMaps.size(); ++i) {
        /* Retrieve the local map */
        const auto& localMapInfo = localMaps.at(i);
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
        if (!this->SaveMapCore(localMapInfo.mMap, poseGraphNodes, saveOptions))
            return false;
    }

    return true;
}

/* Save the grid map constructed from the latest scans */
bool MapSaver::SaveLatestMap(
    const Mapping::GridMapType& latestMap,
    const std::vector<Mapping::PoseGraph::Node>& poseGraphNodes,
    const bool drawTrajectory,
    const int trajectoryNodeIdxMin,
    const int trajectoryNodeIdxMax,
    const bool saveMetadata,
    const std::string& fileName) const
{
    Options saveOptions;
    saveOptions.mDrawTrajectory = drawTrajectory;
    saveOptions.mTrajectoryNodeIdxMin = trajectoryNodeIdxMin;
    saveOptions.mTrajectoryNodeIdxMax = trajectoryNodeIdxMax;
    saveOptions.mDrawScans = false;
    saveOptions.mSaveMetadata = saveMetadata;
    saveOptions.mFileName = fileName + "-latest-map";

    /* Save the map image and metadata */
    return this->SaveMapCore(latestMap, poseGraphNodes, saveOptions);
}

/* Save the map and the scan */
bool MapSaver::SaveLocalMapAndScan(
    const Mapping::LocalMapInfo& localMapInfo,
    const std::vector<Mapping::PoseGraph::Node>& poseGraphNodes,
    const RobotPose2D<double>& scanPose,
    const ScanPtr& scanData,
    bool drawTrajectory,
    bool saveMetadata,
    const std::string& fileName) const
{
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
    return this->SaveMapCore(localMapInfo.mMap, poseGraphNodes, saveOptions);
}

/* Save the latest map and the scan */
bool MapSaver::SaveLatestMapAndScan(
    const Mapping::GridMapType& latestMap,
    const std::vector<Mapping::PoseGraph::Node>& poseGraphNodes,
    const RobotPose2D<double>& scanPose,
    const ScanPtr& scanData,
    const bool drawTrajectory,
    const int trajectoryNodeIdxMin,
    const int trajectoryNodeIdxMax,
    const bool saveMetadata,
    const std::string& fileName) const
{
    Options saveOptions;
    saveOptions.mDrawTrajectory = drawTrajectory;
    saveOptions.mTrajectoryNodeIdxMin = trajectoryNodeIdxMin;
    saveOptions.mTrajectoryNodeIdxMax = trajectoryNodeIdxMax;
    saveOptions.mDrawScans = true;
    saveOptions.mScanPose = scanPose;
    saveOptions.mScanData = scanData;
    saveOptions.mSaveMetadata = saveMetadata;
    saveOptions.mFileName = fileName;

    /* Save the map image */
    return this->SaveMapCore(latestMap, poseGraphNodes, saveOptions);
}

/* Save precomputed grid maps stored in a local grid map */
bool MapSaver::SavePrecomputedGridMaps(
    const Mapping::LocalMapInfo& localMapInfo,
    const std::vector<Mapping::PoseGraph::Node>& poseGraphNodes,
    const std::string& fileName) const
{
    Options saveOptions;
    saveOptions.mDrawTrajectory = false;
    saveOptions.mDrawScans = false;
    saveOptions.mSaveMetadata = false;

    const auto& precompMaps = localMapInfo.mPrecomputedMaps;

    /* Convert the precomputed grid map and save */
    for (const auto& precompMapInfo : precompMaps) {
        const int nodeHeight = precompMapInfo.first;
        const auto& precompMap = precompMapInfo.second;

        /* Create the occupancy grid map */
        Mapping::GridMapType gridMap =
            Mapping::GridMapType::CreateSameSizeMap(precompMap);

        const double unknownVal = precompMap.UnknownValue();
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

        if (!this->SaveMapCore(gridMap, poseGraphNodes, saveOptions))
            return false;
    }

    return true;
}

/* Draw the grid cells to the image */
void MapSaver::DrawMap(
    const gil::rgb8_view_t& mapImageView,
    const Mapping::GridMapType& gridMap,
    const Point2D<int>& patchIdxMin,
    const Point2D<int>& mapSizeInPatches) const
{
    /* Draw the patches to the image */
    for (int y = 0; y < mapSizeInPatches.mY; ++y) {
        for (int x = 0; x < mapSizeInPatches.mX; ++x) {
            const auto& patch = gridMap.PatchAt(
                patchIdxMin.mX + x, patchIdxMin.mY + y);

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
void MapSaver::DrawTrajectory(
    const gil::rgb8_view_t& mapImageView,
    const Mapping::GridMapType& gridMap,
    const std::vector<Mapping::PoseGraph::Node>& poseGraphNodes,
    const Point2D<int>& gridCellIdxMin,
    const Point2D<int>& gridCellIdxMax,
    const int nodeIdxMin,
    const int nodeIdxMax) const
{
    /* Input validity checks */
    assert(nodeIdxMin >= 0 && nodeIdxMin < poseGraphNodes.size());
    assert(nodeIdxMax >= 0 && nodeIdxMax < poseGraphNodes.size());

    /* Draw the trajectory lines to the image */
    const auto& initialNode = poseGraphNodes.at(nodeIdxMin);
    Point2D<int> prevGridCellIdx =
        gridMap.WorldCoordinateToGridCellIndex(
            initialNode.Pose().mX, initialNode.Pose().mY);

    for (int i = nodeIdxMin + 1; i <= nodeIdxMax; ++i) {
        const auto& node = poseGraphNodes.at(i);
        const Point2D<int> gridCellIdx =
            gridMap.WorldCoordinateToGridCellIndex(
                node.Pose().mX, node.Pose().mY);
        const std::vector<Point2D<int>> lineIndices =
            Bresenham(prevGridCellIdx, gridCellIdx);

        for (const auto& interpolatedIdx : lineIndices) {
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
void MapSaver::DrawScan(
    const boost::gil::rgb8_view_t& mapImageView,
    const Mapping::GridMapType& gridMap,
    const Point2D<int>& gridCellIdxMin,
    const Point2D<int>& gridCellIdxMax,
    const RobotPose2D<double>& scanPose,
    const ScanPtr& scanData) const
{
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
bool MapSaver::SaveMapCore(
    const Mapping::GridMapType& gridMap,
    const std::vector<Mapping::PoseGraph::Node>& poseGraphNodes,
    const Options& saveOptions) const
{
    /* Compute the map size to be written to the image */
    Point2D<int> patchIdxMin;
    Point2D<int> patchIdxMax;
    Point2D<int> gridCellIdxMin;
    Point2D<int> gridCellIdxMax;
    Point2D<int> mapSizeInPatches;
    Point2D<int> mapSizeInGridCells;
    gridMap.ComputeActualMapSize(patchIdxMin, patchIdxMax,
                                 gridCellIdxMin, gridCellIdxMax,
                                 mapSizeInPatches, mapSizeInGridCells);

    /* Initialize the image */
    gil::rgb8_image_t mapImage { mapSizeInGridCells.mX,
                                 mapSizeInGridCells.mY };
    const gil::rgb8_view_t& mapImageView = gil::view(mapImage);
    gil::fill_pixels(mapImageView, gil::rgb8_pixel_t(192, 192, 192));

    /* Draw the grid cells to the image */
    this->DrawMap(mapImageView, gridMap,
                  patchIdxMin, mapSizeInPatches);

    /* Draw the trajectory (pose graph nodes) of the robot */
    if (saveOptions.mDrawTrajectory)
        this->DrawTrajectory(mapImageView, gridMap, poseGraphNodes,
                             gridCellIdxMin, gridCellIdxMax,
                             saveOptions.mTrajectoryNodeIdxMin,
                             saveOptions.mTrajectoryNodeIdxMax);

    /* Draw the scans to the image */
    if (saveOptions.mDrawScans)
        this->DrawScan(mapImageView, gridMap,
                       gridCellIdxMin, gridCellIdxMax,
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
        const Point2D<double> topRight =
            gridMap.GridCellIndexToWorldCoordinate(gridCellIdxMax);
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
void MapSaver::SaveMapMetadata(
    const double mapResolution,
    const int patchSize,
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
