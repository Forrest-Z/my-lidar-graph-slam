
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
        auto loopCandidates = this->mLoopSearcher->Search(searchHint);

        /* Perform loop detection using candidates, each of which consists of
         * a collection of pose graph nodes and a local grid map */
        auto loopDetectionQueries =
            pParent->GetLoopDetectionQueries(loopCandidates);
        LoopDetectionResultVector loopDetectionResults;

        this->mLoopDetector->Detect(
            loopDetectionQueries, loopDetectionResults);

        /* Update the local grid maps if modified in the loop detection */
        pParent->UpdatePrecomputedGridMaps(loopDetectionQueries);

        if (loopDetectionResults.empty())
            continue;

        /* Append loop closing constraints using the loop detection results */
        pParent->AppendLoopClosingEdges(loopDetectionResults);

        /* Retrieve all pose graph nodes and edges */
        LocalMapNodeMap localMapNodes;
        ScanNodeMap scanNodes;
        std::vector<PoseGraphEdge> poseGraphEdges;
        pParent->GetPoseGraph(localMapNodes, scanNodes, poseGraphEdges);

        /* Perform pose graph optimization */
        this->mPoseGraphOptimizer->Optimize(
            localMapNodes, scanNodes, poseGraphEdges);
        /* Update pose graph nodes and rebuild grid maps */
        pParent->AfterLoopClosure(localMapNodes, scanNodes);
    }
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
