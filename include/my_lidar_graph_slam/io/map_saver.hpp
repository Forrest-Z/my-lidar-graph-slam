
/* map_saver.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_IO_MAP_SAVER_HPP
#define MY_LIDAR_GRAPH_SLAM_IO_MAP_SAVER_HPP

#include <memory>
#include <string>

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
    using GridMapType = Mapping::GridMapBuilder::GridMapType;
    using ScanPtr = Sensor::ScanDataPtr<double>;
    using PrecomputedMapType = Mapping::GridMapBuilder::PrecomputedMapType;

    /*
     * Options struct holds the settings for MapSaver class
     */
    struct Options
    {
        bool                mDrawTrajectory;
        std::size_t         mTrajectoryNodeIdxMin;
        std::size_t         mTrajectoryNodeIdxMax;
        bool                mDrawScans;
        RobotPose2D<double> mScanPose;
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
    bool SaveMap(const GridMapBuilderPtr& gridMapBuilder,
                 const PoseGraphPtr& poseGraph,
                 const std::string& fileName,
                 bool drawTrajectory,
                 bool saveMetadata) const;
    
    /* Save the pose graph as JSON format */
    bool SavePoseGraph(const PoseGraphPtr& poseGraph,
                       const std::string& fileName) const;
    
    /* Save local maps individually */
    bool SaveLocalMaps(const GridMapBuilderPtr& gridMapBuilder,
                       const PoseGraphPtr& poseGraph,
                       const std::string& fileName,
                       bool drawTrajectory,
                       bool saveMetadata) const;
    
    /* Save the grid map constructed from the latest scans */
    bool SaveLatestMap(const GridMapBuilderPtr& gridMapBuilder,
                       const PoseGraphPtr& poseGraph,
                       const std::string& fileName,
                       bool drawTrajectory,
                       bool saveMetadata) const;
    
    /* Save the map and the scan */
    bool SaveLocalMapAndScan(const GridMapBuilderPtr& gridMapBuilder,
                             const PoseGraphPtr& poseGraph,
                             const std::size_t localMapIdx,
                             const RobotPose2D<double>& scanPose,
                             const ScanPtr& scanData,
                             bool drawTrajectory,
                             bool saveMetadata,
                             const std::string& fileName) const;
    
    /* Save the latest map and the scan */
    bool SaveLatestMapAndScan(const GridMapBuilderPtr& gridMapBuilder,
                              const PoseGraphPtr& poseGraph,
                              const RobotPose2D<double>& scanPose,
                              const ScanPtr& scanData,
                              bool drawTrajectory,
                              bool saveMetadata,
                              const std::string& fileName) const;

    /* Save the precomputed grid maps */
    bool SavePrecomputedGridMaps(const GridMapBuilderPtr& gridMapBuilder,
                                 const PoseGraphPtr& poseGraph,
                                 const int mapIdx,
                                 const std::string& fileName) const;
    
private:
    /* Draw the grid cells to the image */
    void DrawMap(const GridMapType& gridMap,
                 const boost::gil::rgb8_view_t& mapImageView,
                 const Point2D<int>& patchIdxMin,
                 const Point2D<int>& mapSizeInPatches) const;
    
    /* Draw the trajectory lines to the image */
    void DrawTrajectory(const GridMapType& gridMap,
                        const PoseGraphPtr& poseGraph,
                        const boost::gil::rgb8_view_t& mapImageView,
                        const Point2D<int>& gridCellIdxMin,
                        const Point2D<int>& mapSizeInGridCells,
                        const std::size_t nodeIdxMin,
                        const std::size_t nodeIdxMax) const;
    
    /* Draw the scans obtained at the specified node to the image */
    void DrawScan(const GridMapType& gridMap,
                  const boost::gil::rgb8_view_t& mapImageView,
                  const Point2D<int>& gridCellIdxMin,
                  const Point2D<int>& mapSizeInGridCells,
                  const RobotPose2D<double>& scanPose,
                  const ScanPtr& scanData) const;
    
    /* Save the map image and metadata */
    bool SaveMapCore(const GridMapType& gridMap,
                     const PoseGraphPtr& poseGraph,
                     const Options& saveOptions) const;

    /* Save the map metadata as JSON format */
    void SaveMapMetadata(double mapResolution,
                         int patchSize,
                         const Point2D<int>& mapSizeInPatches,
                         const Point2D<int>& mapSizeInGridCells,
                         const Point2D<double>& bottomLeft,
                         const Point2D<double>& topRight,
                         const std::size_t nodeIdxMin,
                         const std::size_t nodeIdxMax,
                         const std::string& fileName) const;
};

} /* namespace IO */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_IO_MAP_SAVER_HPP */
