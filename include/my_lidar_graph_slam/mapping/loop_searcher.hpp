
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
 * LoopClosureCandidateSearchHint struct holds the necessary information
 * for searching the local grid map and the pose graph node, which are used
 * for the loop detection based on exhaustive scan matching
 */
struct LoopClosureCandidateSearchHint final
{
    /* Constructor */
    LoopClosureCandidateSearchHint(
        std::map<int, NodePosition>&& poseGraphNodes,
        std::vector<LocalMapPosition>&& localMapPositions,
        const double accumTravelDist,
        const int latestNodeIdx,
        const int latestLocalMapIdx) :
        mPoseGraphNodes(std::move(poseGraphNodes)),
        mLocalMapPositions(std::move(localMapPositions)),
        mAccumTravelDist(accumTravelDist),
        mLatestNodeIdx(latestNodeIdx),
        mLatestLocalMapIdx(latestLocalMapIdx) { }

    /* Destructor */
    ~LoopClosureCandidateSearchHint() = default;

    /* Information about the pose graph nodes (poses) */
    const std::map<int, NodePosition>   mPoseGraphNodes;
    /* Information about the local map positions
     * Bounding box, grid map resolution, pose graph node indices */
    const std::vector<LocalMapPosition> mLocalMapPositions;
    /* Accumulated travel distance of the robot (current node) */
    const double                        mAccumTravelDist;
    /* Index of the current pose graph node */
    const int                           mLatestNodeIdx;
    /* Index of the local map that contains the current pose graph node */
    const int                           mLatestLocalMapIdx;
};

/*
 * LoopClosurePair struct holds a collection of local map indices and
 * a pose graph node index, which are used for the loop detection.
 * A scan data that is associated to each node is matched against
 * a single local grid map for many-to-one scan matching
 */
struct LoopClosurePair final
{
    /* Constructor */
    LoopClosurePair(std::vector<int>&& nodeIndices,
                    const int localMapIdx,
                    const int localMapNodeIdx) :
        mNodeIndices(std::move(nodeIndices)),
        mLocalMapIdx(localMapIdx),
        mLocalMapNodeIdx(localMapNodeIdx) { }

    /* Destructor */
    ~LoopClosurePair() = default;

    /* Indices of the pose graph nodes */
    const std::vector<int> mNodeIndices;
    /* Index of the local map */
    const int              mLocalMapIdx;
    /* Index of the pose graph node that resides in the local map,
     * which is used for building the loop closing edge
     * Any index of the node inside the local map can be used */
    const int              mLocalMapNodeIdx;
};

/* Vector of the loop closure candidates */
using LoopClosurePairVector = std::vector<LoopClosurePair>;

class LoopClosureCandidate
{
public:
    /* Constructor */
    LoopClosureCandidate() = default;

    /* Destructor */
    virtual ~LoopClosureCandidate() = default;

    /* Find a local map and a pose graph node used for loop closure */
    virtual LoopClosurePairVector Find(
        const LoopClosureCandidateSearchHint& searchHint) = 0;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_SEARCHER_HPP */
