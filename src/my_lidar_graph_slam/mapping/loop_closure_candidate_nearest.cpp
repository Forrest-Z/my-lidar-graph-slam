
/* loop_closure_candidate_nearest.cpp */

#include <cassert>

#include "my_lidar_graph_slam/mapping/loop_closure_candidate_nearest.hpp"

#include "my_lidar_graph_slam/metric/metric.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/* Find a local map and a pose graph node used for loop closure */
LoopClosureCandidate::CandidateVector LoopClosureCandidateNearest::Find(
    const std::shared_ptr<GridMapBuilder>& gridMapBuilder,
    const std::shared_ptr<PoseGraph>& poseGraph,
    const RobotPose2D<double>& robotPose)
{
    const int numOfMaps = static_cast<int>(gridMapBuilder->LocalMaps().size());
    const int numOfNodes = static_cast<int>(poseGraph->Nodes().size());

    /* Check that the grid map and the pose graph are not empty */
    assert(numOfMaps > 0);
    assert(numOfNodes > 0);

    /* LoopClosureCandidateNearest class selects the closest pose graph node
     * and its corresponding local grid map from the current robot pose */
    std::vector<CandidateType> loopClosureCandidates;

    /* Find the index of the local grid map and the pose graph node
     * that may contain loop closure point */
    double nodeDistMinSq = std::pow(this->mNodeDistThreshold, 2.0);
    int candidateMapIdx = numOfMaps;
    int candidateNodeIdx = numOfNodes;
    bool candidateFound = false;
    
    const double accumTravelDist = gridMapBuilder->AccumTravelDist();
    double nodeTravelDist = 0.0;

    RobotPose2D<double> prevPose = poseGraph->NodeAt(0).Pose();

    /* Exclude the latest local grid map */
    for (int mapIdx = 0; mapIdx < numOfMaps - 1; ++mapIdx) {
        /* Retrieve the local grid map */
        const auto& localMapInfo = gridMapBuilder->LocalMapAt(mapIdx);
        const int nodeIdxMin = localMapInfo.mPoseGraphNodeIdxMin;
        const int nodeIdxMax = localMapInfo.mPoseGraphNodeIdxMax;

        for (int nodeIdx = nodeIdxMin; nodeIdx <= nodeIdxMax; ++nodeIdx) {
            /* Retrieve the pose graph node */
            const RobotPose2D<double>& nodePose =
                poseGraph->NodeAt(nodeIdx).Pose();

            /* Calculate the accumulated travel distance */
            nodeTravelDist += Distance(prevPose, nodePose);
            prevPose = nodePose;

            /* Stop the iteration if the travel distance difference falls below
             * the specified threshold */
            if (accumTravelDist - nodeTravelDist < this->mTravelDistThreshold)
                goto Done;

            /* Calculate the distance between the pose graph node and
             * the current pose */
            const double nodeDistSq = SquaredDistance(nodePose, robotPose);

            /* Update the candidate map index and node index */
            if (nodeDistSq < nodeDistMinSq) {
                candidateFound = true;
                nodeDistMinSq = nodeDistSq;
                candidateMapIdx = mapIdx;
                candidateNodeIdx = nodeIdx;
            }
        }
    }

Done:
    /* Set the closest pose graph node index and its corresponding
     * local grid map index */
    if (candidateFound)
        loopClosureCandidates.emplace_back(candidateMapIdx, candidateNodeIdx);

    /* Update metrics */
    if (candidateFound) {
        auto* const pMetric = Metric::MetricManager::Instance();
        auto& distMetrics = pMetric->DistributionMetrics();
        const double nodeDistMin = std::sqrt(nodeDistMinSq);
        distMetrics("LoopDetectionPoseGraphNodeDist")->Observe(nodeDistMin);
    }

    return loopClosureCandidates;
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
