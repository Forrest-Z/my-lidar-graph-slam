
/* map_saver.cpp */

#include "my_lidar_graph_slam/io/map_saver.hpp"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "my_lidar_graph_slam/point.hpp"
#include "my_lidar_graph_slam/pose.hpp"
#include "my_lidar_graph_slam/grid_map/grid_map.hpp"

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
    const RobotPose2D<double>& globalMapPose,
    const Mapping::GridMapType& globalMap,
    const Mapping::ScanNodeMap& scanNodes,
    const std::string& fileName,
    const bool drawTrajectory,
    const bool saveMetadata) const
{
    Assert(!scanNodes.Nodes().empty());

    Options saveOptions {
        drawTrajectory, scanNodes.NodeIdMin(), scanNodes.NodeIdMax(),
        false, RobotPose2D<double>(0.0, 0.0, 0.0), nullptr,
        saveMetadata, fileName };

    /* Global map accommodates all local grid maps acquired */
    /* Save the map image and metadata */
    return this->SaveMapCore(globalMapPose, globalMap,
                             scanNodes, saveOptions);
}

/* Save the pose graph as JSON format */
bool MapSaver::SavePoseGraph(
    const Mapping::LocalMapNodeMap& localMapNodes,
    const Mapping::ScanNodeMap& scanNodes,
    const std::vector<Mapping::PoseGraphEdge>& poseGraphEdges,
    const std::string& fileName) const
{
    pt::ptree jsonPoseGraph;

    /* Write the local map nodes */
    pt::ptree localMapNodesTree;

    for (const auto& [localMapId, localMapNode] : localMapNodes.Nodes()) {
        pt::ptree nodeInfo;

        /* Write the data for the local map node */
        nodeInfo.put("Id", localMapNode.mLocalMapId.mId);
        nodeInfo.put("GlobalPose.X", localMapNode.mGlobalPose.mX);
        nodeInfo.put("GlobalPose.Y", localMapNode.mGlobalPose.mY);
        nodeInfo.put("GlobalPose.Theta", localMapNode.mGlobalPose.mTheta);

        /* Append the local map node */
        localMapNodesTree.push_back(std::make_pair("", nodeInfo));
    }

    jsonPoseGraph.add_child("PoseGraph.LocalMapNodes", localMapNodesTree);

    /* Write the scan nodes */
    pt::ptree scanNodesTree;

    for (const auto& [nodeId, scanNode] : scanNodes.Nodes()) {
        pt::ptree nodeInfo;

        /* Write the data for the scan node */
        nodeInfo.put("Id", scanNode.mNodeId.mId);
        nodeInfo.put("LocalMapId", scanNode.mLocalMapId.mId);
        nodeInfo.put("LocalPose.X", scanNode.mLocalPose.mX);
        nodeInfo.put("LocalPose.Y", scanNode.mLocalPose.mY);
        nodeInfo.put("LocalPose.Theta", scanNode.mLocalPose.mTheta);
        nodeInfo.put("TimeStamp", scanNode.mScanData->TimeStamp());
        nodeInfo.put("GlobalPose.X", scanNode.mGlobalPose.mX);
        nodeInfo.put("GlobalPose.Y", scanNode.mGlobalPose.mY);
        nodeInfo.put("GlobalPose.Theta", scanNode.mGlobalPose.mTheta);

        /* Append the scan node */
        scanNodesTree.push_back(std::make_pair("", nodeInfo));
    }

    jsonPoseGraph.add_child("PoseGraph.ScanNodes", scanNodesTree);

    /* Write the pose graph edges */
    pt::ptree poseGraphEdgesTree;

    for (const auto& edge : poseGraphEdges) {
        pt::ptree edgeInfo;

        /* Write the data for the edge */
        edgeInfo.put("LocalMapNodeId", edge.mLocalMapNodeId.mId);
        edgeInfo.put("ScanNodeId", edge.mScanNodeId.mId);
        edgeInfo.put("EdgeType", static_cast<int>(edge.mEdgeType));
        edgeInfo.put("ConstraintType", static_cast<int>(edge.mConstraintType));
        edgeInfo.put("RelativePose.X", edge.mRelativePose.mX);
        edgeInfo.put("RelativePose.Y", edge.mRelativePose.mY);
        edgeInfo.put("RelativePose.Theta", edge.mRelativePose.mTheta);

        /* Store information matrix elements (upper triangular) */
        pt::ptree infoMatElements;
        const Eigen::Matrix3d& infoMat = edge.mInformationMat;

        for (Eigen::Index i = 0; i < infoMat.rows(); ++i) {
            for (Eigen::Index j = i; j < infoMat.cols(); ++j) {
                pt::ptree matrixElement;
                matrixElement.put_value(infoMat(i, j));
                infoMatElements.push_back(std::make_pair("", matrixElement));
            }
        }

        edgeInfo.add_child("InformationMatrix", infoMatElements);

        /* Append the pose graph edge */
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
    const std::vector<Mapping::LocalMap>& localMaps,
    const Mapping::LocalMapNodeMap& localMapNodes,
    const Mapping::ScanNodeMap& scanNodes,
    const bool drawTrajectory,
    const bool saveMetadata,
    const std::string& fileName) const
{
    Assert(!localMaps.empty());
    Assert(!localMapNodes.Nodes().empty());
    Assert(localMaps.size() == localMapNodes.Nodes().size());

    /* Save local maps individually as PNG images */
    for (const auto& localMap : localMaps) {
        /* Retrieve the local map Id */
        const Mapping::LocalMapId localMapId = localMap.mId;
        /* Retrieve the local map node */
        const auto& localMapNode = localMapNodes.At(localMapId);
        /* Retrieve the global pose of the local map */
        const RobotPose2D<double>& globalPose = localMapNode.mGlobalPose;
        /* Determine the file name */
        const std::string localMapFileName =
            fileName + "-local-map-" + std::to_string(localMapId.mId);

        Options saveOptions {
            drawTrajectory, localMap.mScanNodeIdMin, localMap.mScanNodeIdMax,
            false, RobotPose2D<double>(0.0, 0.0, 0.0), nullptr,
            saveMetadata, localMapFileName };

        /* Save the map image and metadata */
        if (!this->SaveMapCore(globalPose, localMap.mMap,
                               scanNodes, saveOptions))
            return false;
    }

    return true;
}

/* Save the grid map constructed from the latest scans */
bool MapSaver::SaveLatestMap(
    const RobotPose2D<double>& globalMapPose,
    const Mapping::GridMapType& latestMap,
    const Mapping::ScanNodeMap& scanNodes,
    const bool drawTrajectory,
    const Mapping::NodeId scanNodeIdMin,
    const Mapping::NodeId scanNodeIdMax,
    const bool saveMetadata,
    const std::string& fileName) const
{
    Options saveOptions {
        drawTrajectory, scanNodeIdMin, scanNodeIdMax,
        false, RobotPose2D<double>(0.0, 0.0, 0.0), nullptr,
        saveMetadata, fileName + "-latest-map" };

    /* Save the map image and metadata */
    return this->SaveMapCore(globalMapPose, latestMap,
                             scanNodes, saveOptions);
}

/* Save the map and the scan */
bool MapSaver::SaveLocalMapAndScan(
    const RobotPose2D<double>& globalMapPose,
    const Mapping::LocalMap& localMap,
    const Mapping::ScanNodeMap& scanNodes,
    const RobotPose2D<double>& globalScanPose,
    const ScanPtr& scanData,
    bool drawTrajectory,
    bool saveMetadata,
    const std::string& fileName) const
{
    Options saveOptions {
        drawTrajectory, localMap.mScanNodeIdMin, localMap.mScanNodeIdMax,
        true, globalScanPose, scanData,
        saveMetadata, fileName };

    /* Save the map image */
    return this->SaveMapCore(globalMapPose, localMap.mMap,
                             scanNodes, saveOptions);
}

/* Save the latest map and the scan */
bool MapSaver::SaveLatestMapAndScan(
    const RobotPose2D<double>& globalMapPose,
    const Mapping::GridMapType& latestMap,
    const Mapping::ScanNodeMap& scanNodes,
    const RobotPose2D<double>& globalScanPose,
    const ScanPtr& scanData,
    const bool drawTrajectory,
    const Mapping::NodeId scanNodeIdMin,
    const Mapping::NodeId scanNodeIdMax,
    const bool saveMetadata,
    const std::string& fileName) const
{
    Options saveOptions {
        drawTrajectory, scanNodeIdMin, scanNodeIdMax,
        true, globalScanPose, scanData,
        saveMetadata, fileName + "-latest-map" };

    /* Save the map image */
    return this->SaveMapCore(globalMapPose, latestMap,
                             scanNodes, saveOptions);
}

/* Save precomputed grid maps stored in a local grid map */
bool MapSaver::SavePrecomputedGridMaps(
    const RobotPose2D<double>& globalMapPose,
    const Mapping::LocalMap& localMap,
    const Mapping::ScanNodeMap& scanNodes,
    const std::string& fileName) const
{
    Options saveOptions {
        false, Mapping::NodeId(0), Mapping::NodeId(0),
        false, RobotPose2D<double>(0.0, 0.0, 0.0), nullptr,
        false, fileName };

    const auto& precompMaps = localMap.mPrecomputedMaps;

    /* Convert the precomputed grid map and save */
    for (const auto& [nodeHeight, precompMap] : precompMaps) {
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

        if (!this->SaveMapCore(globalMapPose, gridMap,
                               scanNodes, saveOptions))
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
    const RobotPose2D<double>& globalGridMapPose,
    const Mapping::GridMapType& gridMap,
    const Mapping::ScanNodeMap& scanNodes,
    const Point2D<int>& gridCellIdxMin,
    const Point2D<int>& gridCellIdxMax,
    const Mapping::NodeId scanNodeIdMin,
    const Mapping::NodeId scanNodeIdMax) const
{
    /* Make sure that the scan node Ids are with in the valid value range */
    Assert(scanNodeIdMax >= scanNodeIdMin);
    Assert(scanNodeIdMin >= scanNodes.NodeIdMin());
    Assert(scanNodeIdMax <= scanNodes.NodeIdMax());

    /* Get the iterators to the scan nodes with the specified Ids */
    auto firstIt = scanNodes.LowerBound(scanNodeIdMin);
    auto lastIt = scanNodes.UpperBound(scanNodeIdMax);

    /* Make sure that the iterator to the minimum Id is valid */
    Assert(firstIt != scanNodes.Nodes().end());

    /* Draw the trajectory lines to the image */
    const auto& firstNode = firstIt->second;
    const RobotPose2D<double> localFirstNodePose =
        InverseCompound(globalGridMapPose, firstNode.mGlobalPose);
    Point2D<int> prevGridCellIdx =
        gridMap.LocalPosToGridCellIndex(
            localFirstNodePose.mX, localFirstNodePose.mY);

    for (auto nodeIt = std::next(firstIt); nodeIt != lastIt; ++nodeIt) {
        const auto& scanNode = nodeIt->second;
        const RobotPose2D<double> localScanNodePose =
            InverseCompound(globalGridMapPose, scanNode.mGlobalPose);
        const Point2D<int> gridCellIdx =
            gridMap.LocalPosToGridCellIndex(
                localScanNodePose.mX, localScanNodePose.mY);
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
    const RobotPose2D<double>& globalGridMapPose,
    const Mapping::GridMapType& gridMap,
    const Point2D<int>& gridCellIdxMin,
    const Point2D<int>& gridCellIdxMax,
    const RobotPose2D<double>& globalScanPose,
    const ScanPtr& scanData) const
{
    /* Draw the pose where the scan data is obtained to the image */
    const RobotPose2D<double> localScanPose =
        InverseCompound(globalGridMapPose, globalScanPose);
    const Point2D<int> scanPoseIdx =
        gridMap.LocalPosToGridCellIndex(
            localScanPose.mX, localScanPose.mY);

    if (scanPoseIdx.mX >= gridCellIdxMin.mX &&
        scanPoseIdx.mX < gridCellIdxMax.mX - 2 &&
        scanPoseIdx.mY >= gridCellIdxMin.mY &&
        scanPoseIdx.mY < gridCellIdxMax.mX - 2) {
        const int x = scanPoseIdx.mX - gridCellIdxMin.mX;
        const int y = scanPoseIdx.mY - gridCellIdxMin.mY;
        gil::fill_pixels(gil::subimage_view(mapImageView, x, y, 3, 3),
                         gil::rgb8_pixel_t(0, 255, 0));
    }

    const RobotPose2D<double> globalSensorPose =
        Compound(globalScanPose, scanData->RelativeSensorPose());
    const std::size_t numOfScans = scanData->NumOfScans();

    for (std::size_t i = 0; i < numOfScans; ++i) {
        /* Calculate the grid cell index */
        const RobotPose2D<double> globalHitPose =
            scanData->GlobalHitPose(globalSensorPose, i);
        const RobotPose2D<double> localHitPose =
            InverseCompound(globalGridMapPose, globalHitPose);
        const Point2D<int> hitPointIdx =
            gridMap.LocalPosToGridCellIndex(
                localHitPose.mX, localHitPose.mY);

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
    const RobotPose2D<double>& globalGridMapPose,
    const Mapping::GridMapType& gridMap,
    const Mapping::ScanNodeMap& scanNodes,
    const Options& saveOptions) const
{
    /* Compute the map size to be written to the image */
    Point2D<int> patchIdxMin;
    Point2D<int> patchIdxMax;
    Point2D<int> gridCellIdxMin;
    Point2D<int> gridCellIdxMax;
    Point2D<int> mapSizeInPatches;
    Point2D<int> mapSizeInGridCells;
    Point2D<double> mapSizeInMeters;
    gridMap.ComputeActualMapSize(patchIdxMin, patchIdxMax,
                                 gridCellIdxMin, gridCellIdxMax,
                                 mapSizeInPatches, mapSizeInGridCells,
                                 mapSizeInMeters);

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
        this->DrawTrajectory(mapImageView, globalGridMapPose, gridMap,
                             scanNodes, gridCellIdxMin, gridCellIdxMax,
                             saveOptions.mScanNodeIdMin,
                             saveOptions.mScanNodeIdMax);

    /* Draw the scans to the image */
    if (saveOptions.mDrawScans)
        this->DrawScan(mapImageView, globalGridMapPose, gridMap,
                       gridCellIdxMin, gridCellIdxMax,
                       saveOptions.mGlobalScanPose,
                       saveOptions.mScanData);

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
        this->SaveMapMetadata(
            globalGridMapPose, gridMap.Resolution(), gridMap.PatchSize(),
            mapSizeInPatches, mapSizeInGridCells, mapSizeInMeters,
            gridMap.LocalMinPos(), gridMap.LocalMaxPos(),
            saveOptions.mScanNodeIdMin,
            saveOptions.mScanNodeIdMax,
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
    const RobotPose2D<double>& globalGridMapPose,
    const double mapResolution,
    const int patchSize,
    const Point2D<int>& mapSizeInPatches,
    const Point2D<int>& mapSizeInGridCells,
    const Point2D<double>& mapSizeInMeters,
    const Point2D<double>& localMinPos,
    const Point2D<double>& localMaxPos,
    const Mapping::NodeId scanNodeIdMin,
    const Mapping::NodeId scanNodeIdMax,
    const std::string& fileName) const
{
    pt::ptree jsonMetadata;

    /* Write the map metadata */
    jsonMetadata.put("Map.GlobalPose.X", globalGridMapPose.mX);
    jsonMetadata.put("Map.GlobalPose.Y", globalGridMapPose.mY);
    jsonMetadata.put("Map.GlobalPose.Theta", globalGridMapPose.mTheta);
    jsonMetadata.put("Map.Resolution", mapResolution);
    jsonMetadata.put("Map.PatchSize", patchSize);

    jsonMetadata.put("Map.WidthInPatches", mapSizeInPatches.mX);
    jsonMetadata.put("Map.HeightInPatches", mapSizeInPatches.mY);
    jsonMetadata.put("Map.WidthInGridCells", mapSizeInGridCells.mX);
    jsonMetadata.put("Map.HeightInGridCells", mapSizeInGridCells.mY);
    jsonMetadata.put("Map.WidthInMeters", mapSizeInMeters.mX);
    jsonMetadata.put("Map.HeightInMeters", mapSizeInMeters.mY);

    jsonMetadata.put("Map.LocalMinPos.X", localMinPos.mX);
    jsonMetadata.put("Map.LocalMinPos.Y", localMinPos.mY);
    jsonMetadata.put("Map.LocalMaxPos.X", localMaxPos.mX);
    jsonMetadata.put("Map.LocalMaxPos.Y", localMaxPos.mY);

    jsonMetadata.put("Map.ScanNodeIdMin", scanNodeIdMin.mId);
    jsonMetadata.put("Map.ScanNodeIdMax", scanNodeIdMax.mId);

    /* Save the map metadata as JSON format */
    pt::write_json(fileName, jsonMetadata);

    return;
}

} /* namespace IO */
} /* namespace MyLidarGraphSlam */
