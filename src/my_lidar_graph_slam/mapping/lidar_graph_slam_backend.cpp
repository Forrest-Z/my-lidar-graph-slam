
/* lidar_graph_slam_backend.cpp */

#include "my_lidar_graph_slam/mapping/lidar_graph_slam_backend.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/* Constructor */
LidarGraphSlamBackend::LidarGraphSlamBackend(
    const std::shared_ptr<PoseGraphOptimizer>& poseGraphOptimizer,
    const std::shared_ptr<LoopSearcher>& loopSearcher,
    const std::shared_ptr<LoopDetector>& loopDetector) :
    mPoseGraphOptimizer(poseGraphOptimizer),
    mLoopSearcher(loopSearcher),
    mLoopDetector(loopDetector)
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

        /* Find loop candidates */
        const auto searchHint = pParent->GetLoopSearchHint();
        auto loopCandidates = this->mLoopSearcher->Find(searchHint);

        /* Perform loop detection using candidates, each of which consists of
         * a collection of pose graph nodes and a local grid map */
        auto loopDetectionQueries =
            pParent->GetLoopDetectionQueries(loopCandidates);
        LoopDetectionResultVector loopDetectionResults;

        const bool loopDetected = this->mLoopDetector->FindLoop(
            loopDetectionQueries, loopDetectionResults);

        if (!loopDetected)
            continue;

        /* Append loop closing constraints using the loop detection results */
        pParent->AppendLoopClosingEdges(loopDetectionResults);

        /* Perform pose graph optimization and rebuild grid maps */
        pParent->PerformOptimization(this->mPoseGraphOptimizer);
    }
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
