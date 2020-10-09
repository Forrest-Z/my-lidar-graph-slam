
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

    template <typename IdType, typename DataType>
    using IdMap = Mapping::IdMap<IdType, DataType>;

    using NodeId = Mapping::NodeId;
    using ScanNode = Mapping::ScanNode;
    using LocalMapId = Mapping::LocalMapId;
    using LocalMapNode = Mapping::LocalMapNode;
    using PoseGraphEdge = Mapping::PoseGraphEdge;

    /*
     * GridMapDrawOptions struct stores necessary information for saving
     * a grid map as an image, including the grid map itself and its pose
     * in a world coordinate frame
     */
    struct GridMapDrawOptions
    {
        /* Constructor */
        GridMapDrawOptions(const RobotPose2D<double>& globalPose,
                           const Mapping::GridMapType& gridMap) :
            mPose(globalPose), mGridMap(gridMap) { }

        /* Pose of the grid map in a world coordinate frame */
        const RobotPose2D<double>&  mPose;
        /* Grid map */
        const Mapping::GridMapType& mGridMap;
    };

    /*
     * TrajectoryDrawOptions struct stores necessary information for drawing
     * a robot trajectory to an image
     */
    struct TrajectoryDrawOptions
    {
        /* Constructor */
        TrajectoryDrawOptions(const IdMap<NodeId, ScanNode>& scanNodes,
                              const NodeId& nodeIdMin,
                              const NodeId& nodeIdMax) :
            mScanNodes(scanNodes), mIdMin(nodeIdMin), mIdMax(nodeIdMax) { }

        /* Collection of the pose graph scan nodes */
        const IdMap<NodeId, ScanNode>& mScanNodes;
        /* Minimum Id of the scan node */
        const NodeId                   mIdMin;
        /* Maximum Id of the scan node */
        const NodeId                   mIdMax;
    };

    /*
     * ScanDrawOptions struct stores necessary information for drawing
     * a scan data to an image
     */
    struct ScanDrawOptions
    {
        /* Constructor */
        ScanDrawOptions(const ScanNode& scanNode) :
            mScanNode(scanNode) { }

        /* Scan node to be visualized */
        const ScanNode& mScanNode;
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
        const RobotPose2D<double>& gridMapPose,
        const Mapping::GridMapType& gridMap,
        const bool drawTrajectory,
        const IdMap<NodeId, ScanNode>* pScanNodes,
        const NodeId* pNodeIdMin,
        const NodeId* pNodeIdMax,
        const bool saveMetadata,
        const std::string& fileName) const;

    /* Save the pose graph as JSON format */
    bool SavePoseGraph(
        const IdMap<LocalMapId, LocalMapNode>& localMapNodes,
        const IdMap<NodeId, ScanNode>& scanNodes,
        const std::vector<PoseGraphEdge>& poseGraphEdges,
        const std::string& fileName) const;

    /* Save local maps individually */
    bool SaveLocalMaps(
        const IdMap<LocalMapId, Mapping::LocalMap>& localMaps,
        const IdMap<LocalMapId, LocalMapNode>& localMapNodes,
        const bool drawTrajectory,
        const IdMap<NodeId, ScanNode>* pScanNodes,
        const bool saveMetadata,
        const std::string& fileName) const;

    /* Save the map and the scan */
    bool SaveLocalMapAndScan(
        const RobotPose2D<double>& localMapPose,
        const Mapping::LocalMap& localMap,
        const bool drawTrajectory,
        const IdMap<NodeId, ScanNode>* pScanNodes,
        const bool drawScan,
        const ScanNode* pScanNode,
        const bool saveMetadata,
        const std::string& fileName) const;

    /* Save the latest map and the scan */
    bool SaveLatestMapAndScan(
        const RobotPose2D<double>& latestMapPose,
        const Mapping::GridMapType& latestMap,
        const bool drawTrajectory,
        const IdMap<NodeId, ScanNode>* pScanNodes,
        const NodeId* pNodeIdMin,
        const NodeId* pNodeIdMax,
        const bool drawScan,
        const ScanNode* pScanNode,
        const bool saveMetadata,
        const std::string& fileName) const;

    /* Save precomputed grid maps stored in a local grid map */
    bool SavePrecomputedGridMaps(
        const RobotPose2D<double>& globalMapPose,
        const Mapping::LocalMap& localMap,
        const bool saveMetadata,
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
        const IdMap<NodeId, ScanNode>& scanNodes,
        const Point2D<int>& gridCellIdxMin,
        const Point2D<int>& gridCellIdxMax,
        const NodeId scanNodeIdMin,
        const NodeId scanNodeIdMax) const;

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
        const std::unique_ptr<GridMapDrawOptions>& pMapDrawOptions,
        const std::unique_ptr<TrajectoryDrawOptions>& pTrajectoryDrawOptions,
        const std::unique_ptr<ScanDrawOptions>& pScanDrawOptions,
        const bool saveMetadata,
        const std::string& fileName) const;

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
        const std::string& fileName) const;
};

} /* namespace IO */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_IO_MAP_SAVER_HPP */
