
/* loop_detector_empty.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_DETECTOR_EMPTY_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_DETECTOR_EMPTY_HPP

#include "my_lidar_graph_slam/mapping/loop_detector.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

class LoopClosureEmpty final : public LoopClosure
{
public:
    /* Constructor */
    LoopClosureEmpty() = default;
    /* Destructor */
    ~LoopClosureEmpty() = default;

    /* Do nothing for a loop closure */
    bool FindLoop(
        LoopClosureCandidateInfoVector& loopClosureCandidates,
        LoopClosureResultVector& loopClosureResults) override;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_DETECTOR_EMPTY_HPP */
