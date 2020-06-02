
/* lidar_graph_slam.cpp */

#include "my_lidar_graph_slam/mapping/lidar_graph_slam.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

#include <boost/timer/timer.hpp>

#include <Eigen/Core>
#include <Eigen/LU>

#include "my_lidar_graph_slam/io/map_saver.hpp"
#include "my_lidar_graph_slam/metric/metric.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/* Constructor */
LidarGraphSlam::LidarGraphSlam(
    const std::shared_ptr<GridMapBuilder>& gridMapBuilder,
    const ScanMatcherPtr& scanMatcher,
    const std::shared_ptr<PoseGraph>& poseGraph,
    const std::shared_ptr<PoseGraphOptimizer>& poseGraphOptimizer,
    const std::shared_ptr<LoopClosure>& loopClosure,
    int loopClosureInterval,
    const std::shared_ptr<ScanAccumulator>& scanAccumulator,
    const std::shared_ptr<ScanInterpolator>& scanInterpolator,
    const RobotPose2D<double>& initialPose,
    double updateThresholdTravelDist,
    double updateThresholdAngle,
    double updateThresholdTime) :
    mProcessCount(0),
    mGridMapBuilder(gridMapBuilder),
    mScanMatcher(scanMatcher),
    mPoseGraph(poseGraph),
    mPoseGraphOptimizer(poseGraphOptimizer),
    mLoopClosure(loopClosure),
    mLoopClosureInterval(loopClosureInterval),
    mScanAccumulator(scanAccumulator),
    mScanInterpolator(scanInterpolator),
    mInitialPose(initialPose),
    mLastOdomPose(0.0, 0.0, 0.0),
    mAccumulatedTravelDist(0.0),
    mAccumulatedAngle(0.0),
    mLastMapUpdateOdomPose(0.0, 0.0, 0.0),
    mLastMapUpdateTime(0.0),
    mUpdateThresholdTravelDist(updateThresholdTravelDist),
    mUpdateThresholdAngle(updateThresholdAngle),
    mUpdateThresholdTime(updateThresholdTime)
{
}

/* Process scan data and odometry */
bool LidarGraphSlam::ProcessScan(
    const Sensor::ScanDataPtr<double>& rawScanData,
    const RobotPose2D<double>& odomPose)
{
    auto* const pMetric = Metric::MetricManager::Instance();
    auto& distMetrics = pMetric->DistributionMetrics();
    auto& counterMetrics = pMetric->CounterMetrics();

    /* Measure processing time */
    boost::timer::cpu_timer cpuTimer;

    /* Accumulate the travel distance since the last map update */
    this->UpdateAccumTravelDistance(odomPose);

    /* Accumulate scan data */
    if (this->mScanAccumulator != nullptr)
        this->mScanAccumulator->AppendScan(rawScanData);

    /* Check if the map should be updated */
    if (!this->MapUpdateNeeded(rawScanData->TimeStamp()))
        return false;

    /* Interpolate scan data if necessary */
    auto accumulatedScanData = (this->mScanAccumulator != nullptr) ?
        this->mScanAccumulator->ComputeConcatenatedScan() :
        rawScanData;
    auto scanData = (this->mScanInterpolator != nullptr) ?
        this->mScanInterpolator->Interpolate(accumulatedScanData) :
        accumulatedScanData;

    /* Perform Local SLAM */
    this->PerformLocalSlam(scanData, odomPose);

    /* Integrate the scan data into the grid map */
    this->AppendScan();

    /* Perform loop closure if necessary */
    bool loopDetectionPerformed;
    bool loopFound;
    this->PerformLoopClosure(loopDetectionPerformed, loopFound);

    /* Update miscellaneous parameters */
    this->mProcessCount += 1;
    this->mAccumulatedTravelDist = 0.0;
    this->mAccumulatedAngle = 0.0;
    this->mLastMapUpdateOdomPose = odomPose;
    this->mLastMapUpdateTime = scanData->TimeStamp();

    /* Update metrics */
    counterMetrics("ProcessCount")->Increment();
    /* Measure processing time */
    const double processTime = ToMilliSeconds(cpuTimer.elapsed().wall);
    /* Update process time metrics */
    if (this->mProcessCount > 0)
        distMetrics("OverallProcessTime")->Observe(processTime);
    if (loopDetectionPerformed)
        distMetrics("KeyFrameProcessTime")->Observe(processTime);
    if (loopDetectionPerformed && !loopFound)
        distMetrics("ProcessTimeNoLoopClosure")->Observe(processTime);
    if (loopDetectionPerformed && loopFound)
        distMetrics("ProcessTimeWithLoopClosure")->Observe(processTime);

    std::cerr << "Processing frame: " << this->mProcessCount << std::endl;

    return true;
}

/* Accumulate the travel distance since the last map update */
void LidarGraphSlam::UpdateAccumTravelDistance(
    const RobotPose2D<double>& odomPose)
{
    /* Calculate the relative odometry pose */
    const RobotPose2D<double> relOdomPose = (this->mProcessCount == 0) ?
        RobotPose2D<double>(0.0, 0.0, 0.0) :
        InverseCompound(this->mLastOdomPose, odomPose);
    
    this->mLastOdomPose = odomPose;

    /* Accumulate the linear and angular distance */
    this->mAccumulatedTravelDist += Distance(relOdomPose);
    this->mAccumulatedAngle += std::fabs(relOdomPose.mTheta);

    return;
}

/* Check if the grid map should be updated */
bool LidarGraphSlam::MapUpdateNeeded(double currentTime) const
{
    /* Compute the elapsed time since the last map update */
    const double elapsedTime = (this->mProcessCount == 0) ?
        0.0 : currentTime - this->mLastMapUpdateTime;
    
    /* Map is updated only when the robot moved more than the specified
     * distance or the specified time has elapsed since the last map update */
    const bool travelDistThreshold =
        this->mAccumulatedTravelDist >= this->mUpdateThresholdTravelDist;
    const bool angleThreshold =
        this->mAccumulatedAngle >= this->mUpdateThresholdAngle;
    const bool timeThreshold = elapsedTime >= this->mUpdateThresholdTime;
    const bool isFirstScan = this->mProcessCount == 0;
    const bool mapUpdateNeeded =
        travelDistThreshold || angleThreshold || timeThreshold || isFirstScan;
    
    return mapUpdateNeeded;
}

/* Perform Local SLAM */
void LidarGraphSlam::PerformLocalSlam(
    const Sensor::ScanDataPtr<double>& scanData,
    const RobotPose2D<double>& odomPose)
{
    auto* const pMetric = Metric::MetricManager::Instance();
    auto& distMetrics = pMetric->DistributionMetrics();

    /* Update the pose graph */
    if (this->mProcessCount == 0) {
        /* Set the initial pose for the first scan */
        RobotPose2D<double> estimatedPose = this->mInitialPose;
        /* Append the new pose graph node */
        this->mPoseGraph->AppendNode(estimatedPose, scanData);
        return;
    }

    /* Measure processing time */
    boost::timer::cpu_timer localSlamTimer;

    const RobotPose2D<double> relPoseFromLastMapUpdate =
        InverseCompound(this->mLastMapUpdateOdomPose, odomPose);
    const RobotPose2D<double> initialPose =
        Compound(this->mPoseGraph->LatestNode().Pose(),
                 relPoseFromLastMapUpdate);

    /* Perform scan matching against the grid map that contains
     * the latest scans and obtain the result */
    boost::timer::cpu_timer localScanMatchingTimer;
    ScanMatcher::Summary scanMatchSummary;
    this->mScanMatcher->OptimizePose(
        this->mGridMapBuilder->LatestMap(),
        scanData, initialPose, scanMatchSummary);
    distMetrics("LocalSlamScanMatchingTime")->Observe(
        ToMilliSeconds(localScanMatchingTimer.elapsed().wall));

    const RobotPose2D<double>& estimatedPose =
        scanMatchSummary.mEstimatedPose;
    const Eigen::Matrix3d& estimatedCovariance =
        scanMatchSummary.mEstimatedCovariance;

    /* Append the new pose graph node */
    const RobotPose2D<double>& startNodePose =
        this->mPoseGraph->LatestNode().Pose();
    const int startNodeIdx =
        this->mPoseGraph->LatestNode().Index();
    const int endNodeIdx =
        this->mPoseGraph->AppendNode(estimatedPose, scanData);

    /* Two pose graph node indexes must be adjacent
        * since the edge represents the odometry constraint */
    assert(endNodeIdx == startNodeIdx + 1);

    /* Setup the pose graph edge parameters */
    /* Relative pose in the frame of the start node
        * Angular component must be normalized from -pi to pi */
    const RobotPose2D<double> edgeRelPose =
        NormalizeAngle(InverseCompound(startNodePose, estimatedPose));

    /* Covariance matrix must be rotated beforehand since the matrix
        * must represent the covariance in the node frame (not world frame) */
    Eigen::Matrix3d robotFrameCovMat;
    ConvertCovarianceFromWorldToRobot(
        startNodePose, estimatedCovariance, robotFrameCovMat);
    /* Calculate a information matrix by inverting a covariance matrix
        * obtained from the scan matching */
    const Eigen::Matrix3d edgeInfoMat = robotFrameCovMat.inverse();

    /* Append the new pose graph edge for odometric constraint */
    this->mPoseGraph->AppendEdge(startNodeIdx, endNodeIdx,
                                 edgeRelPose, edgeInfoMat);

    /* Measure processing time */
    distMetrics("LocalSlamTime")->Observe(
        ToMilliSeconds(localSlamTimer.elapsed().wall));

    return;
}

/* Integrate the scan data into the grid map */
void LidarGraphSlam::AppendScan()
{
    auto* const pMetric = Metric::MetricManager::Instance();
    auto& distMetrics = pMetric->DistributionMetrics();

    boost::timer::cpu_timer mapUpdateTimer;
    this->mGridMapBuilder->AppendScan(this->mPoseGraph);
    distMetrics("MapUpdateTime")->Observe(
        ToMilliSeconds(mapUpdateTimer.elapsed().wall));
}

/* Perform loop closure */
void LidarGraphSlam::PerformLoopClosure(
    bool& loopDetectionPerformed,
    bool& loopFound)
{
    auto* const pMetric = Metric::MetricManager::Instance();
    auto& distMetrics = pMetric->DistributionMetrics();

    /* Check if the loop detection needs to be performed */
    loopDetectionPerformed =
        this->mProcessCount > this->mLoopClosureInterval &&
        this->mProcessCount % this->mLoopClosureInterval == 0;
    loopFound = false;

    if (!loopDetectionPerformed)
        return;

    /* Perform loop closure (pose graph optimization) */
    /* Relative pose of the pose graph edge */
    RobotPose2D<double> relPose;
    /* Index of the pose graph node for starting pose */
    int startNodeIdx;
    /* Index of the pose graph node for ending pose */
    int endNodeIdx;
    /* Covariance matrix of the pose graph edge */
    Eigen::Matrix3d estimatedCovMat;

    /* Find a loop using a pose graph and local grid maps */
    boost::timer::cpu_timer loopDetectionTimer;
    loopFound = this->mLoopClosure->FindLoop(
        this->mGridMapBuilder, this->mPoseGraph,
        relPose, startNodeIdx, endNodeIdx, estimatedCovMat);
    distMetrics("LoopDetectionTime")->Observe(
        ToMilliSeconds(loopDetectionTimer.elapsed().wall));

    if (!loopFound)
        return;

    /* Setup the pose graph edge parameters */
    /* Relative pose must be normalized beforehand */
    relPose = NormalizeAngle(relPose);

    /* Covariance matrix must be rotated since the matrix
        * must represent the covariance in the node frame */
    const RobotPose2D<double>& startNodePose =
        this->mPoseGraph->NodeAt(startNodeIdx).Pose();
    Eigen::Matrix3d robotFrameCovMat;
    ConvertCovarianceFromWorldToRobot(
        startNodePose, estimatedCovMat, robotFrameCovMat);
    /* Calculate a information matrix by inverting a covariance matrix
        * obtained from the scan matching */
    const Eigen::Matrix3d edgeInfoMat = robotFrameCovMat.inverse();

    /* Append the new pose graph edge for loop closing constraint */
    this->mPoseGraph->AppendEdge(startNodeIdx, endNodeIdx,
                                 relPose, edgeInfoMat);

    /* Perform loop closure */
    boost::timer::cpu_timer optimizationTimer;
    this->mPoseGraphOptimizer->Optimize(this->mPoseGraph);
    distMetrics("PoseGraphOptimizationTime")->Observe(
        ToMilliSeconds(optimizationTimer.elapsed().wall));

    /* Re-create the grid maps */
    boost::timer::cpu_timer regenerateMapTimer;
    this->mGridMapBuilder->AfterLoopClosure(this->mPoseGraph);
    distMetrics("RegenerateMapTime")->Observe(
        ToMilliSeconds(regenerateMapTimer.elapsed().wall));
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
