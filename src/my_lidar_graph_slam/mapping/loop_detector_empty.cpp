
/* loop_detector_empty.cpp */

#include "my_lidar_graph_slam/mapping/loop_detector_empty.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/* Do not perform loop detection */
bool LoopDetectorEmpty::Detect(
    LoopDetectionQueryVector& /* loopDetectionQueries */,
    LoopDetectionResultVector& /* loopDetectionResults */)
{
    /* Do not perform loop detection */
    /* Return false to indicate that the loop detection failed */
    return false;
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
