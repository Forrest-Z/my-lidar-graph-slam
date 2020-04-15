
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

#include <boost/gil/gil_all.hpp>
#include <boost/gil/extension/io/png_io.hpp>

#include "my_lidar_graph_slam/mapping/grid_map_builder.hpp"
#include "my_lidar_graph_slam/mapping/pose_graph.hpp"

namespace MyLidarGraphSlam {
namespace IO {

class MapSaver
{
public:
    /* Type definitions */
    using GridMapBuilderPtr = std::shared_ptr<Mapping::GridMapBuilder>;
    using PoseGraphPtr = std::shared_ptr<Mapping::PoseGraph>;
    using GridMapType = Mapping::GridMapBuilder::GridMapType;

    /* Constructor */
    MapSaver() = default;

    /* Destructor */
    ~MapSaver() = default;

    /* Save the entire map */
    bool SaveMap(const GridMapBuilderPtr& gridMapBuilder,
                 const PoseGraphPtr& poseGraph,
                 const std::string& fileName,
                 bool drawTrajectory) const;
    
    /* Save the pose graph as JSON format */
    bool SavePoseGraph(const PoseGraphPtr& poseGraph,
                       const std::string& fileName) const;
    
    /* Save local maps individually */
    bool SaveLocalMaps(const GridMapBuilderPtr& gridMapBuilder,
                       const PoseGraphPtr& poseGraph,
                       const std::string& fileName,
                       bool drawTrajectory) const;
    
    /* Save the grid map constructed from the latest scans */
    bool SaveLatestMap(const GridMapBuilderPtr& gridMapBuilder,
                       const PoseGraphPtr& poseGraph,
                       const std::string& fileName) const;
    
private:
    /* Determine the actual map size */
    void ComputeActualMapSize(const GridMapType& gridMap,
                              Point2D<int>& patchIdxMin,
                              Point2D<int>& gridCellIdxMin,
                              Point2D<int>& mapSizeInPatches,
                              Point2D<int>& mapSizeInGridCells) const;
    
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
    
    /* Save the map image and metadata */
    bool SaveMapCore(const GridMapType& gridMap,
                     const PoseGraphPtr& poseGraph,
                     std::size_t nodeIdxMin,
                     std::size_t nodeIdxMax,
                     bool drawTrajectory,
                     const std::string& fileName) const;

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
