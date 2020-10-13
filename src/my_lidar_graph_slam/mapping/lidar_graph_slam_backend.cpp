
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

        /* Run a single iteration */
        this->RunStep(pParent);
    }

    /* Perform the last iteration after the SLAM frontend has finished
     * Loop detection and pose graph optimization are performed against
     * the last local grid map appended to the grid map builder */
    this->RunStep(pParent);
}

/* Run a single iteration */
void LidarGraphSlamBackend::RunStep(LidarGraphSlam* const pParent)
{
    /* Find loop candidates */
    const auto searchHint = pParent->GetLoopSearchHint();

    if (searchHint.mLocalMapNodes.empty() || searchHint.mScanNodes.empty())
        return;

    auto loopCandidates = this->mLoopSearcher->Search(searchHint);

    if (loopCandidates.empty())
        return;

    /* Perform loop detection using candidates, each of which consists of
     * a collection of pose graph nodes and a local grid map */
    auto loopDetectionQueries =
        pParent->GetLoopDetectionQueries(loopCandidates);

    LoopDetectionResultVector loopDetectionResults;
    this->mLoopDetector->Detect(
        loopDetectionQueries, loopDetectionResults);

    if (loopDetectionResults.empty())
        return;

    /* Append loop closing constraints using the loop detection results */
    pParent->AppendLoopClosingEdges(loopDetectionResults);

    /* Retrieve the finished pose graph nodes and edges */
    IdMap<LocalMapId, LocalMapNode> localMapNodes;
    IdMap<NodeId, ScanNode> scanNodes;
    std::vector<PoseGraphEdge> poseGraphEdges;
    pParent->GetPoseGraphFinished(localMapNodes, scanNodes, poseGraphEdges);

    /* Perform pose graph optimization */
    this->mPoseGraphOptimizer->Optimize(
        localMapNodes, scanNodes, poseGraphEdges);
    /* Update pose graph nodes and rebuild grid maps */
    pParent->AfterLoopClosure(localMapNodes, scanNodes);
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
