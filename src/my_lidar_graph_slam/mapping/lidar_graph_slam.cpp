
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

    localMapNodes.Clear();
    scanNodes.Clear();
    poseGraphEdges.clear();

    /* Copy the local map nodes */
    for (const auto& [nodeId, mapNode] : this->mPoseGraph->LocalMapNodes())
        localMapNodes.Append(nodeId, mapNode.mGlobalPose);

    /* Copy the scan nodes */
    for (const auto& [nodeId, scanNode] : this->mPoseGraph->ScanNodes())
        scanNodes.Append(nodeId, scanNode.mLocalMapId, scanNode.mLocalPose,
                         scanNode.mScanData, scanNode.mGlobalPose);

    /* Copy the pose graph edges */
    for (const auto& poseGraphEdge : this->mPoseGraph->Edges())
        poseGraphEdges.emplace_back(
            poseGraphEdge.mLocalMapNodeId, poseGraphEdge.mScanNodeId,
            poseGraphEdge.mEdgeType, poseGraphEdge.mConstraintType,
            poseGraphEdge.mRelativePose, poseGraphEdge.mInformationMat);
}

/* Retrieve the pose graph information */
void LidarGraphSlam::GetPoseGraph(
    std::map<LocalMapId, LocalMapData>& localMapNodes,
    std::map<NodeId, ScanNodeData>& scanNodes,
    std::vector<EdgeData>& poseGraphEdges) const
{
    /* Acquire the unique lock */
    std::unique_lock uniqueLock { this->mMutex };

    poseGraphEdges.reserve(this->mPoseGraph->Edges().size());

    /* Setup the local map nodes information */
    for (const auto& [localMapId, localMapNode]:
         this->mPoseGraph->LocalMapNodes()) {
        const auto& localMap = this->mGridMapBuilder->LocalMapAt(localMapId);

        /* Compute the bounding box of the local map in a world frame */
        Point2D<double> globalMinPos;
        Point2D<double> globalMaxPos;
        localMap.mMap.ComputeBoundingBox(
            localMapNode.mGlobalPose, globalMinPos, globalMaxPos);

        /* Append the local map node information */
        localMapNodes.emplace(
            std::piecewise_construct,
            std::forward_as_tuple(localMapId),
            std::forward_as_tuple(localMapId, globalMinPos, globalMaxPos,
                                  localMap.mScanNodeIdMin,
                                  localMap.mScanNodeIdMax,
                                  localMap.mFinished));
    }

    /* Setup the scan nodes information */
    for (const auto& [scanNodeId, scanNode] : this->mPoseGraph->ScanNodes())
        /* Append the scan node information */
        scanNodes.emplace(
            std::piecewise_construct,
            std::forward_as_tuple(scanNodeId),
            std::forward_as_tuple(scanNodeId, scanNode.mGlobalPose));

    /* Setup the pose graph edges information */
    for (const auto& edge : this->mPoseGraph->Edges())
        poseGraphEdges.emplace_back(
            edge.mLocalMapNodeId, edge.mScanNodeId,
            edge.mEdgeType, edge.mConstraintType);
}

/* Retrieve the latest data */
void LidarGraphSlam::GetLatestData(
    RobotPose2D<double>& lastScanPose,
    GridMapType& latestMap,
    RobotPose2D<double>& latestMapPose) const
{
    /* Acquire the unique lock */
    std::unique_lock uniqueLock { this->mMutex };
    /* Set the last pose from the pose graph in a world coordinate frame */
    lastScanPose = this->mPoseGraph->LatestScanNode().mGlobalPose;
    /* Set the latest grid map and its pose in a world coordinate frame */
    latestMap = this->mGridMapBuilder->LatestMap();
    latestMapPose = this->mGridMapBuilder->LatestMapPose();
}

/* Retrieve the necessary information for loop search */
LoopSearchHint LidarGraphSlam::GetLoopSearchHint() const
{
    /* Acquire the unique lock */
    std::unique_lock uniqueLock { this->mMutex };

    std::map<NodeId, ScanNodeData> scanNodes;
    std::map<LocalMapId, LocalMapData> localMapNodes;

    /* Setup the scan nodes information */
    for (const auto& [scanNodeId, scanNode] : this->mPoseGraph->ScanNodes())
        /* Append the scan node information */
        scanNodes.emplace(
            std::piecewise_construct,
            std::forward_as_tuple(scanNodeId),
            std::forward_as_tuple(scanNodeId, scanNode.mGlobalPose));

    /* Setup the local map nodes information */
    for (const auto& [localMapId, localMapNode] :
         this->mPoseGraph->LocalMapNodes()) {
        const auto& localMap = this->mGridMapBuilder->LocalMapAt(localMapId);

        /* Compute the bounding box of the local map in a world frame */
        Point2D<double> globalMinPos;
        Point2D<double> globalMaxPos;
        localMap.mMap.ComputeBoundingBox(
            localMapNode.mGlobalPose, globalMinPos, globalMaxPos);

        /* Append the local map node information */
        localMapNodes.emplace(
            std::piecewise_construct,
            std::forward_as_tuple(localMapId),
            std::forward_as_tuple(localMapId, globalMinPos, globalMaxPos,
                                  localMap.mScanNodeIdMin,
                                  localMap.mScanNodeIdMax,
                                  localMap.mFinished));
    }

    const ScanNode& latestScanNode =
        this->mPoseGraph->LatestScanNode();
    const LocalMapNode& latestLocalMapNode =
        this->mPoseGraph->LatestLocalMapNode();
    const LocalMap& latestLocalMap =
        this->mGridMapBuilder->LatestLocalMap();

    /* Make sure that the Id of the latest local map node stored in PoseGraph
     * and the Id of the latest local map stored in GridMapBuilder are same */
    Assert(latestLocalMapNode.mLocalMapId == latestLocalMap.mId);
    /* Make sure that the latest local map contains the latest scan node */
    Assert(latestScanNode.mLocalMapId == latestLocalMap.mId);
    Assert(latestScanNode.mNodeId >= latestLocalMap.mScanNodeIdMin &&
           latestScanNode.mNodeId <= latestLocalMap.mScanNodeIdMax);

    /* Return the necessary information for loop search */
    return LoopSearchHint {
        std::move(scanNodes), std::move(localMapNodes),
        this->mGridMapBuilder->AccumTravelDist(),
        latestScanNode.mNodeId, latestLocalMapNode.mLocalMapId };
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
        /* Retrieve the detailed information for scan nodes */
        std::vector<ScanNode> scanNodes;
        scanNodes.reserve(loopCandidate.mScanNodeIds.size());

        for (const NodeId& nodeId : loopCandidate.mScanNodeIds)
            scanNodes.push_back(this->mPoseGraph->ScanNodeAt(nodeId));

        /* Retrieve the information for the local grid map */
        LocalMap& localMap = this->mGridMapBuilder->LocalMapAt(
            loopCandidate.mLocalMapId);
        /* Retrieve the information for the local map node */
        LocalMapNode& localMapNode = this->mPoseGraph->LocalMapNodeAt(
            loopCandidate.mLocalMapId);

        /* Append the data needed for loop detection */
        loopDetectionQueries.emplace_back(
            std::move(scanNodes), localMap, localMapNode);
    }

    return loopDetectionQueries;
}

/* Append a first node with an associated scan data, and update the
 * current local grid map and the latest map */
void LidarGraphSlam::AppendFirstNodeAndEdge(
    const RobotPose2D<double>& initialScanPose,
    const Sensor::ScanDataPtr<double>& scanData)
{
    /* Acquire the unique lock */
    std::unique_lock uniqueLock { this->mMutex };
    /* Use a diagonal matrix with sufficiently small values as a covariance
     * matrix to fix the relative pose of the first local map and the first
     * scan node to zero */
    const Eigen::Matrix3d covarianceMatrix =
        Eigen::DiagonalMatrix<double, 3>(1e-9, 1e-9, 1e-9);
    /* Append a new scan data and create a new pose and an edge */
    this->mGridMapBuilder->AppendScan(
        this->mPoseGraph->LocalMapNodes(), this->mPoseGraph->ScanNodes(),
        this->mPoseGraph->Edges(),
        initialScanPose, covarianceMatrix, scanData);
}

/* Append a new pose graph node and an odometry edge, and update the
 * current local grid map and the latest map */
void LidarGraphSlam::AppendNodeAndEdge(
    const RobotPose2D<double>& relativeScanPose,
    const Eigen::Matrix3d& scanPoseCovarianceMatrix,
    const Sensor::ScanDataPtr<double>& scanData)
{
    /* Acquire the unique lock */
    std::unique_lock uniqueLock { this->mMutex };
    /* Append a new scan data and create a new pose and an edge */
    this->mGridMapBuilder->AppendScan(
        this->mPoseGraph->LocalMapNodes(), this->mPoseGraph->ScanNodes(),
        this->mPoseGraph->Edges(),
        relativeScanPose, scanPoseCovarianceMatrix, scanData);
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
        /* The angular component in the pose is normalized beforehand */
        const RobotPose2D<double> relativePose =
            NormalizeAngle(loopDetectionResult.mRelativePose);

        /* Covariance matrix represents the uncertainty of the scan matching */
        /* Covariance matrix should not be rotated since it already represents
         * the covariance of the pose in the map-local coordinate frame and
         * the pose of the local grid map is not changed during the loop
         * detection (SLAM frontend just appends new nodes or edges and does
         * not modify the already existing ones) */

        /* Compute a information matrix by inverting a covariance matrix */
        const Eigen::Matrix3d informationMat =
            loopDetectionResult.mEstimatedCovMat.inverse();

        /* Retrieve the local map information */
        const auto& localMap = this->mGridMapBuilder->LocalMapAt(
            loopDetectionResult.mLocalMapNodeId);
        /* Make sure that the local map is in finished state */
        Assert(localMap.mFinished);
        /* Make sure that the we are creating an edge that represents an
         * inter-local grid map constraint, that is, the local map
         * `localMap` does not contain the scan node `mScanNodeId` */
        Assert(loopDetectionResult.mScanNodeId < localMap.mScanNodeIdMin ||
               loopDetectionResult.mScanNodeId > localMap.mScanNodeIdMax);

        /* Append a new loop closing constraint */
        this->mPoseGraph->AppendEdge(
            loopDetectionResult.mLocalMapNodeId,
            loopDetectionResult.mScanNodeId,
            EdgeType::InterLocalMap, ConstraintType::Loop,
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
        auto& localMap = loopDetectionQuery.mLocalMap;
        auto& oldLocalMap = this->mGridMapBuilder->LocalMapAt(localMap.mId);

        if (localMap.mPrecomputed && !oldLocalMap.mPrecomputed) {
            oldLocalMap.mPrecomputedMaps = std::move(localMap.mPrecomputedMaps);
            oldLocalMap.mPrecomputed = true;
        }
    }
}

/* Update pose graph nodes and rebuild grid maps after loop closure */
void LidarGraphSlam::AfterLoopClosure(
    const LocalMapNodeMap& localMapNodes,
    const ScanNodeMap& scanNodes)
{
    /* Acquire the unique lock */
    std::unique_lock uniqueLock { this->mMutex };

    /* Update local map nodes using the results from the pose graph
     * optimization (edges are not updated since they are constants) */
    for (const auto& [localMapId, localMapNode] : localMapNodes) {
        /* Retrieve the old local map node */
        auto& oldLocalMapNode = this->mPoseGraph->LocalMapNodeAt(localMapId);
        /* Update the node pose */
        oldLocalMapNode.mGlobalPose = localMapNode.mGlobalPose;
    }

    /* Update the scan nodes using the results from the pose graph */
    for (const auto& [scanNodeId, scanNode] : scanNodes) {
        /* Retrieve the old scan node */
        auto& oldScanNode = this->mPoseGraph->ScanNodeAt(scanNodeId);
        /* Update the node pose */
        oldScanNode.mGlobalPose = scanNode.mGlobalPose;
    }

    /* Retrieve the pose and Id of the last node which got updated by the
     * pose graph optimization */
    const auto& lastLocalMapNode = localMapNodes.LatestNode();
    const auto& lastScanNode = scanNodes.LatestNode();
    /* Retrieve the corresponding local map */
    const auto& lastLocalMap = this->mGridMapBuilder->LocalMapAt(
        lastLocalMapNode.mLocalMapId);

    /* Check that only finished local grid maps are optimized */
    /* We just check the last optimized local map `lastLocalMap`, local maps
     * older than this map should be finished */
    Assert(lastLocalMap.mFinished);
    /* The following predicates indicate that the last optimized scan node
     * `lastScanNode` belongs to the last optimized local map `lastLocalMap`
     * and is the last scan of the `lastLocalMap`, which means that no scan
     * that belongs to the unoptimized local grid map got optimized */
    Assert(lastScanNode.mLocalMapId == lastLocalMap.mId);
    Assert(lastScanNode.mNodeId == lastLocalMap.mScanNodeIdMax);

    /* Retrieve the first edge that connects the last optimized local map
     * `lastLocalMap` and the scan node after the last optimized scan node
     * `lastScanNode`, which should be the inter-local grid map odometry
     * constraint */
    const auto odomEdgeIt = std::find_if(
        this->mPoseGraph->Edges().crbegin(),
        this->mPoseGraph->Edges().crend(),
        [&lastLocalMap](const PoseGraphEdge& edge) {
            return edge.mLocalMapNodeId == lastLocalMap.mId &&
                   edge.mScanNodeId > lastLocalMap.mScanNodeIdMax; });

    /* If the above edge is not found, then no new node has been added
     * to the pose graph since the beginning of the loop closure and
     * we do not need to correct pose graph */
    if (odomEdgeIt == this->mPoseGraph->Edges().crend()) {
        this->mGridMapBuilder->AfterLoopClosure(
            this->mPoseGraph->LocalMapNodes(), this->mPoseGraph->ScanNodes());
        return;
    }

    /* Make sure that the above edge is inter-local map odometry constraint */
    Assert(odomEdgeIt->IsIntraLocalMap() &&
           odomEdgeIt->IsOdometryConstraint());

    /* Assume that the pose graph edges after the above edge represent
     * the odometry edges that are not considered in the last loop closure */
    LocalMapId lastLocalMapId = lastLocalMap.mId;
    NodeId lastNodeId = lastLocalMap.mScanNodeIdMax;

    /* Update remaining pose graph nodes after the ending node of the above
     * odometry edge, which are added after the beginning of this loop closure
     * using the relative poses from the odometry edges */
    /* Relative poses between local map nodes and scan nodes inside these
     * local maps are kept */
    for (auto edgeIt = odomEdgeIt.base();
         edgeIt != this->mPoseGraph->Edges().end(); ++edgeIt) {
        /* Retrieve the odometry edge */
        const auto& odomEdge = *edgeIt;
        /* Check what kind of node (local map or scan) should be updated */
        const bool updateScanNode =
            odomEdge.mLocalMapNodeId == lastLocalMapId &&
            odomEdge.mScanNodeId > lastNodeId;
        const bool updateLocalMapNode =
            odomEdge.mLocalMapNodeId > lastLocalMapId &&
            odomEdge.mScanNodeId == lastNodeId;
        /* Make sure that the pose graph edges are sorted in ascending order
         * and there are no duplicate edges */
        Assert(updateLocalMapNode || updateScanNode);

        /* Make sure that we are using the odometry edge */
        Assert(odomEdge.IsOdometryConstraint());

        if (updateScanNode) {
            /* Make sure that we are using the intra-local edge */
            Assert(odomEdge.IsIntraLocalMap());

            /* Retrieve the starting node whose pose is already updated */
            const LocalMapNode& startNode =
                this->mPoseGraph->LocalMapNodeAt(odomEdge.mLocalMapNodeId);
            /* Compute the new node pose */
            const RobotPose2D<double>& startNodePose = startNode.mGlobalPose;
            const RobotPose2D<double> endNodePose =
                Compound(startNodePose, odomEdge.mRelativePose);
            /* Update the node pose */
            ScanNode& endNode =
                this->mPoseGraph->ScanNodeAt(odomEdge.mScanNodeId);
            endNode.mGlobalPose = endNodePose;
        } else if (updateLocalMapNode) {
            /* Make sure that we are using the inter-local edge */
            Assert(odomEdge.IsInterLocalMap());

            /* Retrieve the ending node whose pose is already updated */
            const ScanNode& endNode =
                this->mPoseGraph->ScanNodeAt(odomEdge.mScanNodeId);
            /* Compute the new node pose */
            const RobotPose2D<double>& endNodePose = endNode.mGlobalPose;
            const RobotPose2D<double> startNodePose =
                MoveBackward(endNodePose, odomEdge.mRelativePose);
            /* Update the node pose */
            LocalMapNode& startNode =
                this->mPoseGraph->LocalMapNodeAt(odomEdge.mLocalMapNodeId);
            startNode.mGlobalPose = startNodePose;
        }

        /* Update the last processed edge information */
        lastLocalMapId = odomEdge.mLocalMapNodeId;
        lastNodeId = odomEdge.mScanNodeId;
    }

    /* Rebuild the grid maps */
    this->mGridMapBuilder->AfterLoopClosure(
        this->mPoseGraph->LocalMapNodes(), this->mPoseGraph->ScanNodes());

    return;
}

/* Retrieve a latest map that contains latest scans */
void LidarGraphSlam::GetLatestMap(
    RobotPose2D<double>& globalPose,
    GridMapType& latestMap,
    NodeId& scanNodeIdMin,
    NodeId& scanNodeIdMax) const
{
    /* Acquire the unique lock */
    std::unique_lock uniqueLock { this->mMutex };

    /* Set the pose of the latest map in a world coordinate frame */
    globalPose = this->mGridMapBuilder->LatestMapPose();
    /* Set the latest grid map */
    latestMap = this->mGridMapBuilder->LatestMap();

    /* Set the Id range of the scan node */
    /* Scan data in the scan nodes within the range of `scanNodeIdMin` and
     * `scanNodeIdMax` are used to create a latest grid map */
    scanNodeIdMin = this->mGridMapBuilder->LatestScanIdMin();
    scanNodeIdMax = this->mGridMapBuilder->LatestScanIdMax();

    return;
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
