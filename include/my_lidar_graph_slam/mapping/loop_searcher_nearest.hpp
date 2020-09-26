
/* loop_searcher_nearest.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_SEARCHER_NEAREST_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_SEARCHER_NEAREST_HPP

#include "my_lidar_graph_slam/mapping/loop_searcher.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

class LoopSearcherNearest final : public LoopSearcher
{
public:
    /* Constructor */
    LoopSearcherNearest(double travelDistThreshold,
                        double nodeDistThreshold) :
        LoopSearcher(),
        mTravelDistThreshold(travelDistThreshold),
        mNodeDistThreshold(nodeDistThreshold) { }

    /* Destructor */
    ~LoopSearcherNearest() = default;

    /* Find a local map and a pose graph node used for loop detection */
    LoopCandidateVector Search(
        const LoopSearchHint& searchHint) override;

private:
    /* Travel distance threhsold for loop search
     * Pose graph node that can be traversed from the current node
     * with the travel distance less than this threshold is not considered
     * for loop search */
    double mTravelDistThreshold;
    /* Maximum distance between the current robot pose and
     * the pose of the node which is considered for loop search */
    double mNodeDistThreshold;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_SEARCHER_NEAREST_HPP */
