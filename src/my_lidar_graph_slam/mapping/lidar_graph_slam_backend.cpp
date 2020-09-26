
/* lidar_graph_slam_backend.cpp */

#include "my_lidar_graph_slam/mapping/lidar_graph_slam_backend.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/* Constructor */
LidarGraphSlamBackend::LidarGraphSlamBackend(
    const std::shared_ptr<PoseGraphOptimizer>& poseGraphOptimizer,
    const std::shared_ptr<LoopClosureCandidate>& loopDetectionSearch,
    const std::shared_ptr<LoopClosure>& loopClosure) :
    mPoseGraphOptimizer(poseGraphOptimizer),
    mLoopDetectionSearch(loopDetectionSearch),
    mLoopClosure(loopClosure)
{
}

/* Run loop detection and pose graph optimization */
void LidarGraphSlamBackend::Run(
    LidarGraphSlam* const pParent,
    std::atomic<bool>& stopRequest)
{
    while (!stopRequest.load()) {
        /* Wait for the update from the SLAM frontend */
        pParent->WaitForNotification();

        /* Find loop detection candidates */
        const LoopClosureCandidateSearchHint searchHint =
            pParent->GetLoopDetectionSearchHint();
        LoopClosurePairVector loopDetectionPairs =
            this->mLoopDetectionSearch->Find(searchHint);

        /* Perform loop detection using candidates, each of which consists of
         * a collection of pose graph nodes and a local grid map */
        LoopClosureCandidateInfoVector loopDetectionCandidates =
            pParent->GetLoopDetectionCandidates(loopDetectionPairs);
        LoopClosureResultVector loopDetectionResults;

        const bool loopDetected = this->mLoopClosure->FindLoop(
            loopDetectionCandidates, loopDetectionResults);

        if (!loopDetected)
            continue;

        /* Append loop closing constraints using the loop detection results */
        pParent->AppendLoopClosingEdges(loopDetectionResults);

        /* Perform loop closure and rebuild grid maps */
        pParent->PerformOptimization(this->mPoseGraphOptimizer);
    }
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
