
/* loop_searcher.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_SEARCHER_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_SEARCHER_HPP

#include <map>
#include <memory>
#include <utility>
#include <vector>

#include "my_lidar_graph_slam/pose.hpp"
#include "my_lidar_graph_slam/mapping/grid_map_builder.hpp"
#include "my_lidar_graph_slam/mapping/pose_graph.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/*
 * LoopSearchHint struct holds the necessary information
 * for searching the local grid map and the scan node, which are used
 * for the loop detection based on exhaustive scan matching
 */
struct LoopSearchHint final
{
    /* Constructor */
    LoopSearchHint(
        std::map<NodeId, ScanNodeData>&& scanNodes,
        std::map<LocalMapId, LocalMapData>&& localMapNodes,
        const double accumTravelDist,
        const NodeId latestScanNodeId,
        const LocalMapId latestLocalMapNodeId) :
        mScanNodes(std::move(scanNodes)),
        mLocalMapNodes(std::move(localMapNodes)),
        mAccumTravelDist(accumTravelDist),
        mLatestScanNodeId(latestScanNodeId),
        mLatestLocalMapNodeId(latestLocalMapNodeId) { }

    /* Destructor */
    ~LoopSearchHint() = default;

    /* Information about the scan nodes */
    const std::map<NodeId, ScanNodeData>     mScanNodes;
    /* Information about the local map nodes */
    const std::map<LocalMapId, LocalMapData> mLocalMapNodes;
    /* Accumulated travel distance of the robot (current scan node) */
    const double                             mAccumTravelDist;
    /* Id of the current scan node */
    const NodeId                             mLatestScanNodeId;
    /* Id of the local map that contains the current scan node */
    const LocalMapId                         mLatestLocalMapNodeId;
};

/*
 * LoopCandidate struct holds a collection of a local map Id and
 * scan node Ids, which are used for the loop detection.
 * A scan data that is associated to each scan node is matched against
 * a single local map for a many-to-one scan matching.
 */
struct LoopCandidate final
{
    /* Constructor */
    LoopCandidate(std::vector<NodeId>&& scanNodeIds,
                  const LocalMapId localMapId) :
        mScanNodeIds(std::move(scanNodeIds)),
        mLocalMapId(localMapId) { }

    /* Destructor */
    ~LoopCandidate() = default;

    /* Ids of the scan nodes */
    const std::vector<NodeId> mScanNodeIds;
    /* Id of the local map node */
    const LocalMapId          mLocalMapId;
};

/* Vector of the loop candidates */
using LoopCandidateVector = std::vector<LoopCandidate>;

class LoopSearcher
{
public:
    /* Constructor */
    LoopSearcher() = default;

    /* Destructor */
    virtual ~LoopSearcher() = default;

    /* Find a local map and a scan node for loop detection */
    virtual LoopCandidateVector Search(
        const LoopSearchHint& searchHint) = 0;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_SEARCHER_HPP */
