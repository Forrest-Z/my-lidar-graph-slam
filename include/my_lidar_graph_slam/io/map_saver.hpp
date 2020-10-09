
/* map_saver.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_IO_MAP_SAVER_HPP
#define MY_LIDAR_GRAPH_SLAM_IO_MAP_SAVER_HPP

#include <memory>
#include <string>
#include <vector>

/* Definitions to prevent compile errors
 * int_p_NULL is removed in libpng 1.4 */
#define png_infopp_NULL     (png_infopp)NULL
#define int_p_NULL          (int*)NULL
#define png_bytep_NULL      (png_bytep)NULL

#include <boost/version.hpp>

#if BOOST_VERSION <= 106700
#include <boost/gil/gil_all.hpp>
#include <boost/gil/extension/io/png_io.hpp>
#elif BOOST_VERSION <= 106800
#include <boost/gil/gil_all.hpp>
#include <boost/gil/extension/io/png.hpp>
#else
#include <boost/gil.hpp>
#include <boost/gil/extension/io/png.hpp>
#endif

#include "my_lidar_graph_slam/point.hpp"
#include "my_lidar_graph_slam/pose.hpp"
#include "my_lidar_graph_slam/mapping/grid_map_builder.hpp"
#include "my_lidar_graph_slam/mapping/pose_graph.hpp"
#include "my_lidar_graph_slam/sensor/sensor_data.hpp"

namespace MyLidarGraphSlam {
namespace IO {

class MapSaver
{
public:
    /* Type definitions */
    using GridMapBuilderPtr = std::shared_ptr<Mapping::GridMapBuilder>;
    using PoseGraphPtr = std::shared_ptr<Mapping::PoseGraph>;
    using ScanPtr = Sensor::ScanDataPtr<double>;

    /*
     * Options struct holds the settings for MapSaver class
     */
    struct Options
    {
        /* Constructor */
        Options(const bool drawTrajectory,
                const Mapping::NodeId scanNodeIdMin,
                const Mapping::NodeId scanNodeIdMax,
                const bool drawScans,
                const RobotPose2D<double>& globalScanPose,
                const ScanPtr& scanData,
                const bool saveMetadata,
                const std::string& fileName) :
            mDrawTrajectory(drawTrajectory),
            mScanNodeIdMin(scanNodeIdMin),
            mScanNodeIdMax(scanNodeIdMax),
            mDrawScans(drawScans),
            mGlobalScanPose(globalScanPose),
            mScanData(scanData),
            mSaveMetadata(saveMetadata),
            mFileName(fileName) { }

        bool                mDrawTrajectory;
        Mapping::NodeId     mScanNodeIdMin;
        Mapping::NodeId     mScanNodeIdMax;
        bool                mDrawScans;
        RobotPose2D<double> mGlobalScanPose;
        ScanPtr             mScanData;
        bool                mSaveMetadata;
        std::string         mFileName;
    };

private:
    /* Constructor */
    MapSaver() = default;
    /* Destructor */
    ~MapSaver() = default;

public:
    /* Copy constructor (disabled) */
    MapSaver(const MapSaver&) = delete;
    /* Copy assignment operator (disabled) */
    MapSaver& operator=(const MapSaver&) = delete;
    /* Move constructor (disabled) */
    MapSaver(MapSaver&&) = delete;
    /* Move assignment operator (disabled) */
    MapSaver& operator=(MapSaver&&) = delete;

    /* Get the MapSaver singleton instance */
    static MapSaver* Instance();

    /* Save the entire map */
    bool SaveMap(
        const RobotPose2D<double>& globalMapPose,
        const Mapping::GridMapType& globalMap,
        const Mapping::IdMap<Mapping::NodeId,
            Mapping::ScanNode>& scanNodes,
        const std::string& fileName,
        const bool drawTrajectory,
        const bool saveMetadata) const;

    /* Save the pose graph as JSON format */
    bool SavePoseGraph(
        const Mapping::IdMap<Mapping::LocalMapId,
            Mapping::LocalMapNode>& localMapNodes,
        const Mapping::IdMap<Mapping::NodeId,
            Mapping::ScanNode>& scanNodes,
        const std::vector<Mapping::PoseGraphEdge>& poseGraphEdges,
        const std::string& fileName) const;

    /* Save local maps individually */
    bool SaveLocalMaps(
        const std::vector<Mapping::LocalMap>& localMaps,
        const Mapping::IdMap<Mapping::LocalMapId,
            Mapping::LocalMapNode>& localMapNodes,
        const Mapping::IdMap<Mapping::NodeId,
            Mapping::ScanNode>& scanNodes,
        const bool drawTrajectory,
        const bool saveMetadata,
        const std::string& fileName) const;

    /* Save the grid map constructed from the latest scans */
    bool SaveLatestMap(
        const RobotPose2D<double>& globalMapPose,
        const Mapping::GridMapType& latestMap,
        const Mapping::IdMap<Mapping::NodeId,
            Mapping::ScanNode>& scanNodes,
        const bool drawTrajectory,
        const Mapping::NodeId scanNodeIdMin,
        const Mapping::NodeId scanNodeIdMax,
        const bool saveMetadata,
        const std::string& fileName) const;

    /* Save the map and the scan */
    bool SaveLocalMapAndScan(
        const RobotPose2D<double>& globalMapPose,
        const Mapping::LocalMap& localMap,
        const Mapping::IdMap<Mapping::NodeId,
            Mapping::ScanNode>& scanNodes,
        const RobotPose2D<double>& globalScanPose,
        const ScanPtr& scanData,
        const bool drawTrajectory,
        const bool saveMetadata,
        const std::string& fileName) const;

    /* Save the latest map and the scan */
    bool SaveLatestMapAndScan(
        const RobotPose2D<double>& globalMapPose,
        const Mapping::GridMapType& latestMap,
        const Mapping::IdMap<Mapping::NodeId,
            Mapping::ScanNode>& scanNodes,
        const RobotPose2D<double>& globalScanPose,
        const ScanPtr& scanData,
        const bool drawTrajectory,
        const Mapping::NodeId scanNodeIdMin,
        const Mapping::NodeId scanNodeIdMax,
        const bool saveMetadata,
        const std::string& fileName) const;

    /* Save precomputed grid maps stored in a local grid map */
    bool SavePrecomputedGridMaps(
        const RobotPose2D<double>& globalMapPose,
        const Mapping::LocalMap& localMap,
        const Mapping::IdMap<Mapping::NodeId,
            Mapping::ScanNode>& scanNodes,
        const std::string& fileName) const;

private:
    /* Draw the grid cells to the image */
    void DrawMap(
        const boost::gil::rgb8_view_t& mapImageView,
        const Mapping::GridMapType& gridMap,
        const Point2D<int>& patchIdxMin,
        const Point2D<int>& mapSizeInPatches) const;

    /* Draw the trajectory lines to the image */
    void DrawTrajectory(
        const boost::gil::rgb8_view_t& mapImageView,
        const RobotPose2D<double>& globalGridMapPose,
        const Mapping::GridMapType& gridMap,
        const Mapping::IdMap<Mapping::NodeId,
            Mapping::ScanNode>& scanNodes,
        const Point2D<int>& gridCellIdxMin,
        const Point2D<int>& gridCellIdxMax,
        const Mapping::NodeId scanNodeIdMin,
        const Mapping::NodeId scanNodeIdMax) const;

    /* Draw the scans obtained at the specified node to the image */
    void DrawScan(
        const boost::gil::rgb8_view_t& mapImageView,
        const RobotPose2D<double>& globalGridMapPose,
        const Mapping::GridMapType& gridMap,
        const Point2D<int>& gridCellIdxMin,
        const Point2D<int>& gridCellIdxMax,
        const RobotPose2D<double>& globalScanPose,
        const ScanPtr& scanData) const;

    /* Save the map image and metadata */
    bool SaveMapCore(
        const RobotPose2D<double>& globalGridMapPose,
        const Mapping::GridMapType& gridMap,
        const Mapping::IdMap<Mapping::NodeId,
            Mapping::ScanNode>& scanNodes,
        const Options& saveOptions) const;

    /* Save the map metadata as JSON format */
    void SaveMapMetadata(
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
        const std::string& fileName) const;
};

} /* namespace IO */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_IO_MAP_SAVER_HPP */
