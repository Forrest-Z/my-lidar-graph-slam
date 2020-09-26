
/* loop_searcher_nearest.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_SEARCHER_NEAREST_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_SEARCHER_NEAREST_HPP

#include "my_lidar_graph_slam/mapping/loop_searcher.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

class LoopClosureCandidateNearest final : public LoopClosureCandidate
{
public:
    /* Constructor */
    LoopClosureCandidateNearest(double travelDistThreshold,
                                double nodeDistThreshold) :
        LoopClosureCandidate(),
        mTravelDistThreshold(travelDistThreshold),
        mNodeDistThreshold(nodeDistThreshold) { }

    /* Destructor */
    ~LoopClosureCandidateNearest() = default;

    /* Find a local map and a pose graph node used for loop closure */
    LoopClosurePairVector Find(
        const LoopClosureCandidateSearchHint& searchHint) override;

private:
    /* Travel distance threhsold for loop closure
     * Pose graph node that can be traversed from the current node
     * with the travel distance less than this threshold is not considered
     * for loop closure */
    double mTravelDistThreshold;
    /* Maximum distance between the current robot pose and
     * the pose of the node which is considered for loop closure */
    double mNodeDistThreshold;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_SEARCHER_NEAREST_HPP */
