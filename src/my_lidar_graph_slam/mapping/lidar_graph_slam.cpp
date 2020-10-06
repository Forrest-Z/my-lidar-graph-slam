
/* lidar_graph_slam.cpp */

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/LU>

#include "my_lidar_graph_slam/mapping/lidar_graph_slam.hpp"
#include "my_lidar_graph_slam/mapping/lidar_graph_slam_backend.hpp"
#include "my_lidar_graph_slam/mapping/lidar_graph_slam_frontend.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/* Constructor */
LidarGraphSlam::LidarGraphSlam(
    const std::shared_ptr<FrontendType>& slamFrontend,
    const std::shared_ptr<BackendType>& slamBackend,
    const std::shared_ptr<GridMapBuilder>& gridMapBuilder,
    const std::shared_ptr<PoseGraph>& poseGraph) :
    mFrontend(slamFrontend),
    mBackend(slamBackend),
    mBackendThread(nullptr),
    mBackendStopRequest(false),
    mBackendNotifyCond(),
    mBackendNotify(false),
    mGridMapBuilder(gridMapBuilder),
    mPoseGraph(poseGraph)
{
}

/* Process scan data and odometry */
bool LidarGraphSlam::ProcessScan(
    const Sensor::ScanDataPtr<double>& rawScanData,
    const RobotPose2D<double>& odomPose)
{
    /* Process the latest scan data and odometry information in frontend */
    return this->mFrontend->ProcessScan(this, rawScanData, odomPose);
}

/* Retrieve the total number of the processed input data */
int LidarGraphSlam::ProcessCount() const
{
    return this->mFrontend->ProcessCount();
}

/* Retrieve the full pose graph information */
void LidarGraphSlam::GetPoseGraph(
    LocalMapNodeMap& localMapNodes,
    ScanNodeMap& scanNodes,
    std::vector<PoseGraphEdge>& poseGraphEdges) const
{
    /* Acquire the unique lock */
    std::unique_lock uniqueLock { this->mMutex };

    /* Copy the local map nodes and scan nodes */
    localMapNodes = this->mPoseGraph->LocalMapNodes();
    scanNodes = this->mPoseGraph->ScanNodes();
    /* Copy the pose graph edges */
    poseGraphEdges = this->mPoseGraph->Edges();
}

/* Retrieve the pose graph information */
void LidarGraphSlam::GetPoseGraph(
    std::map<int, NodePosition>& poseGraphNodes,
    std::vector<EdgeConnection>& poseGraphEdges) const
{
    /* Acquire the unique lock */
    std::unique_lock uniqueLock { this->mMutex };

    poseGraphEdges.reserve(this->mPoseGraph->Edges().size());

    /* Set the pose graph nodes information */
    for (const auto& node : this->mPoseGraph->Nodes())
        poseGraphNodes.insert(std::make_pair(
            node.Index(), NodePosition(node.Index(), node.Pose())));

    /* Set the pose graph edges information */
    for (const auto& edge : this->mPoseGraph->Edges())
        poseGraphEdges.emplace_back(
            edge.StartNodeIndex(), edge.EndNodeIndex(),
            edge.IsOdometricConstraint());
}

/* Retrieve the latest pose and the latest map */
void LidarGraphSlam::GetLatestPoseAndMap(
    RobotPose2D<double>& latestPose,
    GridMapType& latestMap) const
{
    /* Acquire the unique lock */
    std::unique_lock uniqueLock { this->mMutex };
    /* Retrieve the latest pose from the pose graph */
    latestPose = this->mPoseGraph->LatestNode().Pose();
    /* Copy the latest map from the grid map builder */
    latestMap = this->mGridMapBuilder->LatestMap();
}

/* Retrieve the necessary information for loop search */
LoopSearchHint LidarGraphSlam::GetLoopSearchHint() const
{
    /* Acquire the unique lock */
    std::unique_lock uniqueLock { this->mMutex };

    std::map<int, NodePosition> poseGraphNodes;
    std::map<int, LocalMapPosition> localMapPositions;

    /* Set the pose graph nodes information */
    for (const auto& node : this->mPoseGraph->Nodes()) {
        NodePosition nodePosition { node.Index(), node.Pose() };
        poseGraphNodes.insert(std::make_pair(
            node.Index(), std::move(nodePosition)));
    }

    /* Set the positions of the local grid maps */
    for (const auto& localMap : this->mGridMapBuilder->LocalMaps()) {
        LocalMapPosition localMapPos {
            localMap.mIdx,
            localMap.mMap.MinPos(),
            localMap.mMap.MaxPos(),
            localMap.mPoseGraphNodeIdxMin,
            localMap.mPoseGraphNodeIdxMax,
            localMap.mFinished };
        localMapPositions.insert(std::make_pair(
            localMap.mIdx, std::move(localMapPos)));
    }

    /* Make sure that the index of the latest local map
     * contains the latest pose graph node */
    /* Latest local map is the last element of the local maps,
     * and not the latest map, which is built from the latest several scans */
    const int latestNodeIdx =
        this->mPoseGraph->LatestNode().Index();
    const int latestLocalMapIdx =
        this->mGridMapBuilder->LocalMaps().back().mIdx;
    const auto& latestLocalMap =
        this->mGridMapBuilder->LocalMaps().back();

    assert(latestNodeIdx >= latestLocalMap.mPoseGraphNodeIdxMin &&
           latestNodeIdx <= latestLocalMap.mPoseGraphNodeIdxMax);

    /* Return the necessary information for loop search */
    return LoopSearchHint {
        std::move(poseGraphNodes),
        std::move(localMapPositions),
        this->mGridMapBuilder->AccumTravelDist(),
        latestNodeIdx,
        latestLocalMapIdx };
}

/* Retrieve the necessary information for loop detection */
LoopDetectionQueryVector LidarGraphSlam::GetLoopDetectionQueries(
    const LoopCandidateVector& loopCandidates) const
{
    /* Acquire the unique lock */
    std::unique_lock uniqueLock { this->mMutex };

    LoopDetectionQueryVector loopDetectionQueries;
    loopDetectionQueries.reserve(loopCandidates.size());

    /* Setup the data needed for loop detection */
    for (const auto& loopCandidate : loopCandidates) {
        /* Retrieve the information for pose graph nodes consisting of
         * poses, indices, and scan data, each of which is matched against
         * the local grid map */
        std::vector<PoseGraph::Node> nodes;
        nodes.reserve(loopCandidate.mNodeIndices.size());

        for (const int& nodeIdx : loopCandidate.mNodeIndices)
            nodes.push_back(this->mPoseGraph->NodeAt(nodeIdx));

        /* Retrieve the information for the local grid map */
        LocalMapInfo localMapInfo =
            this->mGridMapBuilder->LocalMapAt(loopCandidate.mLocalMapIdx);
        /* Retrieve the pose graph node that is inside the local grid map */
        PoseGraph::Node localMapNode =
            this->mPoseGraph->NodeAt(loopCandidate.mLocalMapNodeIdx);

        /* Append the data needed for loop detection */
        loopDetectionQueries.emplace_back(
            std::move(nodes), localMapInfo, localMapNode);
    }

    return loopDetectionQueries;
}

/* Append a new pose graph node with an associated scan data */
void LidarGraphSlam::AppendNode(
    const RobotPose2D<double>& nodePose,
    const Sensor::ScanDataPtr<double>& scanData)
{
    /* Acquire the unique lock */
    std::unique_lock uniqueLock { this->mMutex };
    /* Append a new pose graph node */
    this->mPoseGraph->AppendNode(nodePose, scanData);
}

/* Append a new pose graph node and an odometry edge
 * from a scan matching result */
void LidarGraphSlam::AppendOdometryNodeAndEdge(
    const Sensor::ScanDataPtr<double>& scanData,
    const RobotPose2D<double>& edgeRelativePose,
    const Eigen::Matrix3d& edgeCovarianceMatrix)
{
    /* Acquire the unique lock */
    std::unique_lock uniqueLock { this->mMutex };

    /* Setup two robot poses */
    const RobotPose2D<double> startNodePose =
        this->mPoseGraph->LatestNode().Pose();
    /* Compute a new node pose using the latest node pose here,
     * since the pose of the latest node (starting node of the odometry edge)
     * might have been modified by the loop closure,
     * which is performed in the SLAM backend */
    const RobotPose2D<double> newNodePose =
        Compound(startNodePose, edgeRelativePose);

    /* Append the new pose graph node */
    const int startNodeIdx =
        this->mPoseGraph->LatestNode().Index();
    const int endNodeIdx =
        this->mPoseGraph->AppendNode(newNodePose, scanData);

    /* Two pose graph node indices must be adjacent
     * since the edge represents the odometry constraint */
    assert(endNodeIdx == startNodeIdx + 1);

    /* Setup the pose graph edge parameters */
    /* Relative pose in the frame of the start node
     * Angular component must be normalized from -pi to pi */
    const RobotPose2D<double> edgeRelPose =
        NormalizeAngle(edgeRelativePose);

    /* Covariance matrix must be rotated beforehand since the matrix
     * must represent the covariance in the node frame (not world frame) */
    const Eigen::Matrix3d robotFrameCovMat =
        ConvertCovarianceFromWorldToRobot(
            startNodePose, edgeCovarianceMatrix);
    /* Calculate a information matrix by inverting a covariance matrix
     * obtained from the scan matching */
    const Eigen::Matrix3d edgeInfoMat = robotFrameCovMat.inverse();

    /* Append the new pose graph edge for odometric constraint */
    this->mPoseGraph->AppendEdge(startNodeIdx, endNodeIdx,
                                 edgeRelPose, edgeInfoMat);
}

/* Append new loop closing edges */
void LidarGraphSlam::AppendLoopClosingEdges(
    const LoopDetectionResultVector& loopDetectionResults)
{
    /* Acquire the unique lock */
    std::unique_lock uniqueLock { this->mMutex };

    /* Append new loop closing edges for each loop detection result */
    for (const auto& loopDetectionResult : loopDetectionResults) {
        /* Setup the pose graph edge parameters */
        /* Relative pose must be normalized beforehand */
        const RobotPose2D<double> relativePose =
            NormalizeAngle(loopDetectionResult.mRelativePose);

        /* Covariance matrix represents the uncertainty of the scan matching */
        /* Covariance matrix must be rotated since loop closing edge needs
         * the covariance matrix in the local robot frame */
        const Eigen::Matrix3d robotFrameCovMat =
            ConvertCovarianceFromWorldToRobot(
                loopDetectionResult.mStartNodePose,
                loopDetectionResult.mEstimatedCovMat);
        /* Compute a information matrix by inverting a covariance matrix */
        const Eigen::Matrix3d informationMat =
            robotFrameCovMat.inverse();

        /* Append a new loop closing constraint */
        this->mPoseGraph->AppendEdge(
            loopDetectionResult.mStartNodeIdx,
            loopDetectionResult.mEndNodeIdx,
            relativePose, informationMat);
    }
}

/* Update the coarser grid maps if modified in the loop detection process */
void LidarGraphSlam::UpdatePrecomputedGridMaps(
    LoopDetectionQueryVector& loopDetectionQueries)
{
    /* Acquire the unique lock */
    std::unique_lock uniqueLock { this->mMutex };

    /* Update the precomputed coarser grid maps if necessary */
    for (auto& loopDetectionQuery : loopDetectionQueries) {
        auto& localMapInfo = loopDetectionQuery.mLocalMapInfo;
        const int localMapIdx = localMapInfo.mIdx;
        auto& oldLocalMapInfo = this->mGridMapBuilder->LocalMapAt(localMapIdx);

        if (localMapInfo.mPrecomputed && !oldLocalMapInfo.mPrecomputed) {
            oldLocalMapInfo.mPrecomputedMaps =
                std::move(localMapInfo.mPrecomputedMaps);
            oldLocalMapInfo.mPrecomputed = true;
        }
    }
}

/* Update the grid map according to the modified pose graph */
bool LidarGraphSlam::UpdateGridMap()
{
    /* Acquire the unique lock */
    std::unique_lock uniqueLock { this->mMutex };
    /* Update the grid map according to the modified pose graph */
    const bool localMapCreated =
        this->mGridMapBuilder->AppendScan(this->mPoseGraph);
    /* Return whether the new local map is created */
    return localMapCreated;
}

/* Update pose graph nodes and rebuild grid maps after loop closure */
void LidarGraphSlam::AfterLoopClosure(
    const std::vector<PoseGraph::Node>& poseGraphNodes)
{
    /* Acquire the unique lock */
    std::unique_lock uniqueLock { this->mMutex };

    /* Update pose graph nodes using the results from the pose graph
     * optimization, edges are not updated since they are constants */
    for (std::size_t i = 0; i < poseGraphNodes.size(); ++i) {
        /* Retrieve the old and new pose graph node */
        auto& oldNode = this->mPoseGraph->NodeAt(i);
        const auto& newNode = poseGraphNodes.at(i);
        /* Update the node pose */
        oldNode.Pose() = newNode.Pose();
    }

    /* Retrieve the pose and index of the last node which is updated by the
     * pose graph optimization */
    const int optimizedLastNodeIdx = poseGraphNodes.back().Index();
    RobotPose2D<double> nodePose = poseGraphNodes.back().Pose();

    /* Retrieve the odometry edge which has the above node as a starting node
     * Nodes after the ending node of this edge is not optimized */
    const auto odomEdgeIt = std::find_if(
        this->mPoseGraph->Edges().crbegin(),
        this->mPoseGraph->Edges().crend(),
        [optimizedLastNodeIdx](const PoseGraph::Edge& edge) {
            return edge.StartNodeIndex() == optimizedLastNodeIdx; });
    /* This odometry edge must be found */
    assert(odomEdgeIt != this->mPoseGraph->Edges().rend());
    /* Assume that the pose graph edges after the below index represent
     * the odometry edges that are not considered in the last loop closure */
    const std::size_t odomEdgeIdx = std::distance(
        this->mPoseGraph->Edges().begin(), odomEdgeIt.base()) - 1;

    /* Update remaining pose graph nodes after the ending node of the above
     * odometry edge, which are added after the beginning of this loop closure
     * using the relative poses from the odometry edges */
    for (std::size_t edgeIdx = odomEdgeIdx;
         edgeIdx < this->mPoseGraph->Edges().size(); ++edgeIdx) {
        /* Retrieve the odometry edge */
        const auto& odomEdge = this->mPoseGraph->EdgeAt(edgeIdx);
        /* Make sure that we are handling the odometry edge */
        assert(odomEdge.IsOdometricConstraint());
        /* Compute the new node pose */
        nodePose = Compound(nodePose, odomEdge.RelativePose());
        /* Update the node pose */
        auto& odomNode = this->mPoseGraph->NodeAt(odomEdge.EndNodeIndex());
        odomNode.Pose() = nodePose;
    }

    /* Rebuild the grid maps */
    this->mGridMapBuilder->AfterLoopClosure(this->mPoseGraph);
}

/* Retrieve a latest map that contains latest scans */
GridMapType LidarGraphSlam::GetLatestMap(
    int& poseGraphNodeIdxMin, int& poseGraphNodeIdxMax) const
{
    /* Acquire the unique lock */
    std::unique_lock uniqueLock { this->mMutex };
    /* Set the index range of the pose graph node */
    poseGraphNodeIdxMin = this->mGridMapBuilder->LatestScanIdxMin();
    poseGraphNodeIdxMax = this->mGridMapBuilder->LatestScanIdxMax();
    /* Return a latest map */
    return this->mGridMapBuilder->LatestMap();
}

/* Build a global map that contains all local grid maps acquired */
GridMapType LidarGraphSlam::GetGlobalMap(
    int& poseGraphNodeIdxMin, int& poseGraphNodeIdxMax) const
{
    /* Acquire the unique lock */
    std::unique_lock uniqueLock { this->mMutex };
    /* Set the index range of the pose graph node */
    poseGraphNodeIdxMin = this->mPoseGraph->Nodes().front().Index();
    poseGraphNodeIdxMax = this->mPoseGraph->Nodes().back().Index();
    /* Return a global map */
    return this->mGridMapBuilder->ConstructGlobalMap(this->mPoseGraph);
}

/* Start the SLAM backend */
void LidarGraphSlam::StartBackend()
{
    /* Make sure that the thread is not created */
    assert(this->mBackendThread == nullptr);

    /* Create the worker thread for the SLAM backend */
    this->mBackendThread = std::make_shared<std::thread>(
        [this]() { this->mBackend->Run(
            this, std::ref(this->mBackendStopRequest)); });
}

/* Stop the SLAM backend */
void LidarGraphSlam::StopBackend()
{
    if (this->mBackendThread == nullptr)
        return;

    /* Acquire the unique lock */
    std::unique_lock uniqueLock { this->mMutex };

    /* Notify the SLAM backend to terminate */
    this->mBackendNotify = true;
    this->mBackendStopRequest = true;
    this->mBackendNotifyCond.notify_all();

    /* Release the lock */
    uniqueLock.unlock();

    /* Wait for the SLAM backend to terminate */
    if (this->mBackendThread->joinable())
        this->mBackendThread->join();

    /* Destroy the worker thread */
    this->mBackendThread.reset();
}

/* Notify the SLAM backend */
void LidarGraphSlam::NotifyBackend()
{
    /* Acquire the unique lock */
    std::unique_lock uniqueLock { this->mMutex };
    /* Notify the SLAM backend */
    this->mBackendNotify = true;
    this->mBackendNotifyCond.notify_all();
}

/* Wait for the notification from the SLAM frontend */
void LidarGraphSlam::WaitForNotification()
{
    /* Acquire the unique lock */
    std::unique_lock uniqueLock { this->mMutex };
    /* Wait for the notification from the SLAM frontend */
    this->mBackendNotifyCond.wait(uniqueLock,
        [this]() { return this->mBackendNotify; });
    /* Update the flag for notification */
    this->mBackendNotify = false;
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
