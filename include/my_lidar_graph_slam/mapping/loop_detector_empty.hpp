
/* loop_detector_empty.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_DETECTOR_EMPTY_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_DETECTOR_EMPTY_HPP

#include "my_lidar_graph_slam/mapping/loop_detector.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

class LoopDetectorEmpty final : public LoopDetector
{
public:
    /* Constructor */
    LoopDetectorEmpty() = default;
    /* Destructor */
    ~LoopDetectorEmpty() = default;

    /* Do nothing for a loop detection */
    bool FindLoop(
        LoopDetectionQueryVector& loopDetectionQueries,
        LoopDetectionResultVector& loopDetectionResults) override;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_DETECTOR_EMPTY_HPP */
