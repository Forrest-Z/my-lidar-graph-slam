
/* lidar_graph_slam.cpp */

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

/* Retrieve the pose graph information */
void LidarGraphSlam::GetPoseGraphData(
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

/* Retrieve the data for loop detection candidate search */
LoopClosureCandidateSearchHint
    LidarGraphSlam::GetLoopDetectionSearchHint() const
{
    /* Acquire the unique lock */
    std::unique_lock uniqueLock { this->mMutex };

    std::map<int, NodePosition> poseGraphNodes;
    std::vector<LocalMapPosition> localMapPositions;

    /* Set the pose graph nodes information */
    for (const auto& node : this->mPoseGraph->Nodes())
        poseGraphNodes.insert(std::make_pair(
            node.Index(), NodePosition(node.Index(), node.Pose())));

    /* Set the positions of the local grid maps */
    for (const auto& localMap : this->mGridMapBuilder->LocalMaps())
        localMapPositions.emplace_back(
            localMap.mMap.MinPos(),
            localMap.mMap.MaxPos(),
            localMap.mPoseGraphNodeIdxMin,
            localMap.mPoseGraphNodeIdxMax,
            localMap.mFinished);

    /* Make sure that the index of the latest local map
     * contains the latest pose graph node */
    /* Latest local map is the last element of the local maps,
     * and not the latest map, which is built from the latest several scans */
    const int latestNodeIdx =
        this->mPoseGraph->LatestNode().Index();
    const int latestLocalMapIdx = static_cast<int>(
        this->mGridMapBuilder->LocalMaps().size());
    const auto& latestLocalMap =
        this->mGridMapBuilder->LocalMaps().back();

    assert(latestNodeIdx >= latestLocalMap.mPoseGraphNodeIdxMin &&
           latestNodeIdx <= latestLocalMap.mPoseGraphNodeIdxMax);

    /* Return the data for loop detection candidate search */
    return LoopClosureCandidateSearchHint {
        std::move(poseGraphNodes),
        std::move(localMapPositions),
        this->mGridMapBuilder->AccumTravelDist(),
        latestNodeIdx,
        latestLocalMapIdx };
}

/* Retrieve the data for loop detection */
LoopClosureCandidateInfoVector
    LidarGraphSlam::GetLoopDetectionCandidates(
    const LoopClosurePairVector& loopDetectionPairs) const
{
    /* Acquire the unique lock */
    std::unique_lock uniqueLock { this->mMutex };

    LoopClosureCandidateInfoVector loopDetectionCandidates;
    loopDetectionCandidates.reserve(loopDetectionPairs.size());

    /* Setup the data needed for loop detection */
    for (const auto& loopDetectionPair : loopDetectionPairs) {
        /* Retrieve the information for pose graph nodes consisting of
         * poses, indices, and scan data, each of which is matched against
         * the local grid map */
        std::vector<PoseGraph::Node> candidateNodes;
        candidateNodes.reserve(loopDetectionPair.mNodeIndices.size());

        for (const int& nodeIdx : loopDetectionPair.mNodeIndices)
            candidateNodes.push_back(this->mPoseGraph->NodeAt(nodeIdx));

        /* Retrieve the information for the local grid map */
        GridMapBuilder::LocalMapInfo localMapInfo =
            this->mGridMapBuilder->LocalMapAt(loopDetectionPair.mLocalMapIdx);
        /* Retrieve the pose graph node that is inside the local grid map */
        PoseGraph::Node localMapNode =
            this->mPoseGraph->NodeAt(loopDetectionPair.mLocalMapNodeIdx);

        /* Append the data needed for loop detection */
        loopDetectionCandidates.emplace_back(
            std::move(candidateNodes), localMapInfo, localMapNode);
    }

    return loopDetectionCandidates;
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

/* Append a new pose graph node and odometry edge */
void LidarGraphSlam::AppendOdometryNodeAndEdge(
    const RobotPose2D<double>& nodePose,
    const Eigen::Matrix3d& poseCovMat,
    const Sensor::ScanDataPtr<double>& scanData)
{
    /* Acquire the unique lock */
    std::unique_lock uniqueLock { this->mMutex };

    /* Setup two robot poses */
    const RobotPose2D<double> startNodePose =
        this->mPoseGraph->LatestNode().Pose();

    /* Append the new pose graph node */
    const int startNodeIdx =
        this->mPoseGraph->LatestNode().Index();
    const int endNodeIdx =
        this->mPoseGraph->AppendNode(nodePose, scanData);

    /* Two pose graph node indices must be adjacent
     * since the edge represents the odometry constraint */
    assert(endNodeIdx == startNodeIdx + 1);

    /* Setup the pose graph edge parameters */
    /* Relative pose in the frame of the start node
     * Angular component must be normalized from -pi to pi */
    const RobotPose2D<double> edgeRelPose =
        NormalizeAngle(InverseCompound(startNodePose, nodePose));

    /* Covariance matrix must be rotated beforehand since the matrix
     * must represent the covariance in the node frame (not world frame) */
    Eigen::Matrix3d robotFrameCovMat;
    ConvertCovarianceFromWorldToRobot(
        startNodePose, poseCovMat, robotFrameCovMat);
    /* Calculate a information matrix by inverting a covariance matrix
     * obtained from the scan matching */
    const Eigen::Matrix3d edgeInfoMat = robotFrameCovMat.inverse();

    /* Append the new pose graph edge for odometric constraint */
    this->mPoseGraph->AppendEdge(startNodeIdx, endNodeIdx,
                                 edgeRelPose, edgeInfoMat);
}

/* Append new loop closing edges */
void LidarGraphSlam::AppendLoopClosingEdges(
    const LoopClosureResultVector& loopDetectionResults)
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
        Eigen::Matrix3d robotFrameCovMat;
        ConvertCovarianceFromWorldToRobot(
            loopDetectionResult.mStartNodePose,
            loopDetectionResult.mEstimatedCovMat,
            robotFrameCovMat);
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

/* Perform pose graph optimization and rebuild grid maps */
void LidarGraphSlam::PerformOptimization(
    const std::shared_ptr<PoseGraphOptimizer>& poseGraphOptimizer)
{
    /* Acquire the unique lock */
    std::unique_lock uniqueLock { this->mMutex };
    /* Perform loop closure */
    poseGraphOptimizer->Optimize(this->mPoseGraph);
    /* Rebuild the grid maps */
    this->mGridMapBuilder->AfterLoopClosure(this->mPoseGraph);
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
