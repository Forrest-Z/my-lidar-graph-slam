
/* loop_detector_empty.cpp */

#include "my_lidar_graph_slam/mapping/loop_detector_empty.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/* Do not perform loop detection */
void LoopDetectorEmpty::Detect(
    LoopDetectionQueryVector& /* loopDetectionQueries */,
    LoopDetectionResultVector& loopDetectionResults)
{
    /* Do not perform loop detection */
    /* Clear the loop detection results */
    /* Empty result indicates that the loop detection failed */
    loopDetectionResults.clear();
    return;
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
