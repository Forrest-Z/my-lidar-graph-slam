
/* loop_closure_empty.cpp */

#include "my_lidar_graph_slam/mapping/loop_closure_empty.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/* Find a loop and return a loop constraint */
bool LoopClosureEmpty::FindLoop(
    LoopClosureCandidateInfoVector& /* loopClosureCandidates */,
    LoopClosureResultVector& /* loopClosureResults */)
{
    /* Do not perform loop closure */
    /* Return false to indicate that the loop detection failed */
    return false;
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
