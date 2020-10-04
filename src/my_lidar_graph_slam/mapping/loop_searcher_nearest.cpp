
/* loop_searcher_nearest.cpp */

#include <cassert>
#include <numeric>

#include "my_lidar_graph_slam/mapping/loop_searcher_nearest.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/* Find a local map and a scan node used for loop detection */
LoopCandidateVector LoopSearcherNearest::Search(
    const LoopSearchHint& searchHint)
{
    const auto& scanNodes = searchHint.mScanNodes;
    const auto& localMapNodes = searchHint.mLocalMapNodes;

    /* Check that the local map nodes and scan nodes are not empty */
    Assert(!scanNodes.empty());
    Assert(!localMapNodes.empty());

    /* Retrieve the pose of the current scan node */
    const RobotPose2D<double>& robotPose =
        scanNodes.at(searchHint.mLatestScanNodeId).mGlobalPose;

    /* Find the Id of the local grid map and the scan node
     * that may contain loop closure point */
    double nodeDistMinSq = std::pow(this->mNodeDistThreshold, 2.0);
    int candidateMapId = LocalMapId::Invalid;
    int candidateNodeId = NodeId::Invalid;

    /* Retrieve the accumulated travel distance of the robot */
    const double accumTravelDist = searchHint.mAccumTravelDist;

    double nodeTravelDist = 0.0;
    bool isFirstNode = true;
    RobotPose2D<double> prevPose;

    /* Traverse all the local grid maps except the last unfinished one */
    const auto firstMapIt = localMapNodes.begin();
    const auto lastMapIt = std::prev(localMapNodes.end());

    for (auto mapIt = firstMapIt; mapIt != lastMapIt; ++mapIt) {
        /* Retrieve the local map node */
        const auto& localMapId = mapIt->first;
        const auto& localMapNode = mapIt->second;

        /* Make sure that the local map is finished */
        Assert(localMapNode.mFinished);

        /* Retrieve the Id range of the scan nodes in this local map */
        const NodeId scanNodeIdMin = localMapNode.mScanNodeIdMin;
        const NodeId scanNodeIdMax = localMapNode.mScanNodeIdMax;

        /* Retrieve two iterators pointing the scan nodes in this local map */
        const auto firstIt = scanNodes.find(scanNodeIdMin);
        const auto lastIt = scanNodes.find(scanNodeIdMax);

        /* Make sure that the iterators are valid */
        Assert(firstIt != scanNodes.end());
        Assert(lastIt != scanNodes.end());

        for (auto nodeIt = firstIt; nodeIt != std::next(lastIt); ++nodeIt) {
            /* Retrieve the scan node */
            const auto& scanNodeId = nodeIt->first;
            const auto& scanNode = nodeIt->second;

            /* Retrieve the global pose of the scan node */
            const RobotPose2D<double>& nodePose = scanNode.mGlobalPose;

            /* Compute the accumulated travel distance */
            nodeTravelDist += isFirstNode ? 0.0 : Distance(prevPose, nodePose);
            prevPose = nodePose;
            isFirstNode = false;

            /* Stop the iteration if the travel distance difference falls below
             * the specified threshold */
            if (accumTravelDist - nodeTravelDist < this->mTravelDistThreshold)
                goto Done;

            /* Calculate the distance between the scan node and
             * the current pose */
            const double nodeDistSq = SquaredDistance(nodePose, robotPose);

            /* Update the candidate map index and node index */
            if (nodeDistSq < nodeDistMinSq) {
                nodeDistMinSq = nodeDistSq;
                candidateMapId = localMapId.mId;
                candidateNodeId = scanNodeId.mId;
            }
        }
    }

Done:
    /* LoopSearcherNearest class selects the closest scan node
     * and its corresponding local grid map from the current robot pose */
    LoopCandidateVector loopCandidates;

    /* Return an empty collection of candidates if not found */
    if (candidateMapId == LocalMapId::Invalid ||
        candidateNodeId == NodeId::Invalid)
        return loopCandidates;

    /* Set the scan nodes around the current scan node */
    const auto& latestLocalMap =
        localMapNodes.at(searchHint.mLatestLocalMapNodeId);
    const auto firstNodeIt =
        scanNodes.find(latestLocalMap.mScanNodeIdMin);
    const auto lastNodeIt =
        scanNodes.find(latestLocalMap.mScanNodeIdMax);
    const auto latestNodeIt =
        scanNodes.find(searchHint.mLatestScanNodeId);

    Assert(firstNodeIt != scanNodes.end());
    Assert(lastNodeIt != scanNodes.end());
    Assert(latestNodeIt != scanNodes.end());

    const int distToFirstCandidate = std::min(
        static_cast<int>(std::distance(firstNodeIt, latestNodeIt)),
        this->mNumOfCandidateNodes);
    const int distToLastCandidate = std::min(
        static_cast<int>(std::distance(latestNodeIt, lastNodeIt)),
        this->mNumOfCandidateNodes);
    const auto firstCandidateNodeIt =
        std::prev(latestNodeIt, distToFirstCandidate);
    const auto lastCandidateNodeIt =
        std::next(latestNodeIt, distToLastCandidate);
    const int numOfActualCandidateNodes =
        std::distance(firstCandidateNodeIt, lastCandidateNodeIt) + 1;

    std::vector<NodeId> nodeIds;
    nodeIds.resize(numOfActualCandidateNodes);

    for (auto nodeIt = firstCandidateNodeIt;
         nodeIt != std::next(lastCandidateNodeIt); ++nodeIt)
        nodeIds.push_back(nodeIt->first);

    /* Set the loop candidate information */
    loopCandidates.emplace_back(std::move(nodeIds), candidateMapId);

    return loopCandidates;
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
