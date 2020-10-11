
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

    /* Retrieve the pose of the scan node in the last finished local map */
    const RobotPose2D<double>& robotPose =
        scanNodes.at(searchHint.mLastFinishedScanId).mGlobalPose;

    /* Find the Id of the local grid map and the scan node
     * that may contain loop closure point */
    double nodeDistMinSq = std::pow(this->mNodeDistThreshold, 2.0);
    LocalMapId candidateMapId { LocalMapId::Invalid };
    NodeId candidateNodeId { NodeId::Invalid };

    /* Retrieve the accumulated travel distance of the robot */
    const double accumTravelDist = searchHint.mAccumTravelDist;

    double nodeTravelDist = 0.0;
    bool isFirstNode = true;
    RobotPose2D<double> prevPose;

    /* Traverse all finished local grid maps except the last one */
    const auto firstMapIt = localMapNodes.begin();
    const auto lastMapIt = std::prev(localMapNodes.end());
    const auto mapRange = localMapNodes.RangeFromIterator(
        firstMapIt, lastMapIt);

    for (const auto& [localMapId, localMapNode] : mapRange) {
        /* Make sure that the local map is finished */
        Assert(localMapNode.mFinished);

        /* Retrieve two iterators pointing the scan nodes in this local map */
        const auto firstIt = scanNodes.find(localMapNode.mScanNodeIdMin);
        const auto lastIt = scanNodes.find(localMapNode.mScanNodeIdMax);
        /* Make sure that the iterators are valid */
        Assert(firstIt != scanNodes.end());
        Assert(lastIt != scanNodes.end());

        const auto scanRange = scanNodes.RangeFromIterator(
            firstIt, std::next(lastIt));

        for (const auto& [scanNodeId, scanNode] : scanRange) {
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
                candidateMapId = localMapId;
                candidateNodeId = scanNodeId;
            }
        }
    }

Done:
    /* Return an empty collection of candidates if not found */
    if (candidateMapId.mId == LocalMapId::Invalid ||
        candidateNodeId.mId == NodeId::Invalid)
        return LoopCandidateVector { };

    /* Set the scan nodes around the current scan node */
    const auto& lastFinishedMap =
        localMapNodes.at(searchHint.mLastFinishedMapId);
    const auto& firstScanIt =
        scanNodes.find(lastFinishedMap.mScanNodeIdMin);
    const auto& lastScanIt =
        scanNodes.find(lastFinishedMap.mScanNodeIdMax);
    const auto& lastFinishedScanIt =
        scanNodes.find(searchHint.mLastFinishedScanId);

    /* Make sure that these iterators point to the valid elements */
    Assert(firstScanIt != scanNodes.end());
    Assert(lastScanIt != scanNodes.end());
    Assert(lastFinishedScanIt != scanNodes.end());

    /* Make sure that the scan node in the last finished local map
     * `lastFinishedScanIt` is actually inside the last finished local map */
    Assert(lastFinishedScanIt->mId >= firstScanIt->mId &&
           lastFinishedScanIt->mId <= lastScanIt->mId);

    const auto endScanIt = std::next(lastScanIt);

    const int distToFirstCandidate = std::min(
        static_cast<int>(std::distance(firstScanIt, lastFinishedScanIt)),
        this->mNumOfCandidateNodes / 2);
    const auto firstCandidateNodeIt =
        std::prev(lastFinishedScanIt, distToFirstCandidate);
    const int distToEndCandidate = std::min(
        static_cast<int>(std::distance(firstCandidateNodeIt, endScanIt)),
        this->mNumOfCandidateNodes);
    const auto endCandidateNodeIt =
        std::next(firstCandidateNodeIt, distToEndCandidate);
    const int numOfActualCandidateNodes =
        std::distance(firstCandidateNodeIt, endCandidateNodeIt);

    const auto candidateNodeRange = scanNodes.RangeFromIterator(
        firstCandidateNodeIt, endCandidateNodeIt);

    std::vector<NodeId> nodeIds;
    nodeIds.reserve(numOfActualCandidateNodes);

    for (const auto& [scanNodeId, scanNode] : candidateNodeRange)
        nodeIds.push_back(scanNodeId);

    /* LoopSearcherNearest class selects the closest scan node
     * and its corresponding local grid map from the current robot pose */
    LoopCandidateVector loopCandidates;

    /* Set the loop candidate information */
    loopCandidates.emplace_back(std::move(nodeIds), candidateMapId);

    return loopCandidates;
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
