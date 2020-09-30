
/* loop_searcher_nearest.cpp */

#include <cassert>
#include <numeric>

#include "my_lidar_graph_slam/mapping/loop_searcher_nearest.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/* Find a local map and a pose graph node used for loop detection */
LoopCandidateVector LoopSearcherNearest::Search(
    const LoopSearchHint& searchHint)
{
    const auto& poseGraphNodes = searchHint.mPoseGraphNodes;
    const auto& localMapPositions = searchHint.mLocalMapPositions;
    const int numOfNodes = static_cast<int>(poseGraphNodes.size());
    const int numOfMaps = static_cast<int>(localMapPositions.size());

    /* Check that the grid map and the pose graph are not empty */
    assert(numOfMaps > 0);
    assert(numOfNodes > 0);

    /* Retrieve the index of the current node */
    const int latestNodeIdx = searchHint.mLatestNodeIdx;
    /* Retrieve the pose of the current node */
    const RobotPose2D<double>& robotPose =
        poseGraphNodes.at(latestNodeIdx).mPose;

    /* Find the index of the local grid map and the pose graph node
     * that may contain loop closure point */
    double nodeDistMinSq = std::pow(this->mNodeDistThreshold, 2.0);
    int candidateMapIdx = numOfMaps;
    int candidateNodeIdx = numOfNodes;

    /* Retrieve the accumulated travel distance of the robot */
    const double accumTravelDist = searchHint.mAccumTravelDist;

    double nodeTravelDist = 0.0;
    RobotPose2D<double> prevPose = poseGraphNodes.at(0).mPose;

    /* Exclude the latest (unfinished) local grid map */
    for (int mapIdx = 0; mapIdx < numOfMaps - 1; ++mapIdx) {
        /* Retrieve the local grid map */
        const auto& localMapPosition = localMapPositions.at(mapIdx);
        const int nodeIdxMin = localMapPosition.mPoseGraphNodeIdxMin;
        const int nodeIdxMax = localMapPosition.mPoseGraphNodeIdxMax;

        /* Make sure that the local grid map is finished */
        assert(localMapPosition.mFinished);

        for (int nodeIdx = nodeIdxMin; nodeIdx <= nodeIdxMax; ++nodeIdx) {
            /* Retrieve the pose graph node */
            const RobotPose2D<double>& nodePose =
                poseGraphNodes.at(nodeIdx).mPose;

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
                nodeDistMinSq = nodeDistSq;
                candidateMapIdx = mapIdx;
                candidateNodeIdx = nodeIdx;
            }
        }
    }

Done:
    /* LoopSearcherNearest class selects the closest pose graph node
     * and its corresponding local grid map from the current robot pose */
    LoopCandidateVector loopCandidates;

    /* Return an empty collection of candidates if not found */
    if (candidateMapIdx < 0 || candidateMapIdx >= numOfMaps ||
        candidateNodeIdx < 0 || candidateNodeIdx >= numOfNodes)
        return loopCandidates;

    /* Set the pose graph nodes around the closest pose graph node */
    const auto& latestLocalMap =
        searchHint.mLocalMapPositions.at(searchHint.mLatestLocalMapIdx);
    const int nodeIdxMin = std::max(
        latestLocalMap.mPoseGraphNodeIdxMin,
        latestNodeIdx - this->mNumOfCandidateNodes);
    const int nodeIdxMax = std::min(
        latestLocalMap.mPoseGraphNodeIdxMax,
        latestNodeIdx + this->mNumOfCandidateNodes);

    std::vector<int> nodeIndices;
    nodeIndices.resize(nodeIdxMax - nodeIdxMin + 1);
    std::iota(std::begin(nodeIndices), std::end(nodeIndices), nodeIdxMin);

    loopCandidates.emplace_back(
        std::move(nodeIndices), candidateMapIdx, candidateNodeIdx);

    return loopCandidates;
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
