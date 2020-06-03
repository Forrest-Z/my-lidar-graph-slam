
/* loop_closure_real_time_correlative.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_CLOSURE_REAL_TIME_CORRELATIVE_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_CLOSURE_REAL_TIME_CORRELATIVE_HPP

#include "my_lidar_graph_slam/mapping/cost_function.hpp"
#include "my_lidar_graph_slam/mapping/score_function.hpp"
#include "my_lidar_graph_slam/mapping/loop_closure.hpp"
#include "my_lidar_graph_slam/mapping/loop_closure_candidate_nearest.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

class LoopClosureRealTimeCorrelative final : public LoopClosure
{
public:
    /* Constructor */
    LoopClosureRealTimeCorrelative(const CostFuncPtr& costFunc,
                                   double travelDistThreshold,
                                   double nodeDistThreshold,
                                   int lowResolution,
                                   double rangeX,
                                   double rangeY,
                                   double rangeTheta,
                                   double scanRangeMax,
                                   double scoreThreshold) :
        mCostFunc(costFunc),
        mLoopClosureCandidate(travelDistThreshold, nodeDistThreshold),
        mLowResolution(lowResolution),
        mRangeX(rangeX),
        mRangeY(rangeY),
        mRangeTheta(rangeTheta),
        mScanRangeMax(scanRangeMax),
        mScoreThreshold(scoreThreshold) { }

    /* Destructor */
    ~LoopClosureRealTimeCorrelative() = default;

    /* Find a loop and return a loop constraint */
    bool FindLoop(std::shared_ptr<GridMapBuilder>& gridMapBuilder,
                  const std::shared_ptr<PoseGraph>& poseGraph,
                  RobotPose2D<double>& relPose,
                  int& startNodeIdx,
                  int& endNodeIdx,
                  Eigen::Matrix3d& estimatedCovMat) override;

private:
    /* Compute the search step */
    void ComputeSearchStep(
        const GridMapBuilder::GridMapType& gridMap,
        const Sensor::ScanDataPtr<double>& scanData,
        double& stepX,
        double& stepY,
        double& stepTheta) const;

    /* Compute the grid cell indices for scan points */
    void ComputeScanIndices(
        const GridMapBuilder::PrecomputedMapType& precompMap,
        const RobotPose2D<double>& sensorPose,
        const Sensor::ScanDataPtr<double>& scanData,
        std::vector<Point2D<int>>& scanIndices) const;

    /* Compute the scan matching score based on the already projected
     * scan points (indices) and index offsets */
    double ComputeScore(
        const GridMapBase<double>& gridMap,
        const std::vector<Point2D<int>>& scanIndices,
        const int offsetX,
        const int offsetY) const;

    /* Evaluate the matching score using high-resolution grid map */
    void EvaluateHighResolutionMap(
        const GridMapBase<double>& gridMap,
        const std::vector<Point2D<int>>& scanIndices,
        const int offsetX,
        const int offsetY,
        const int offsetTheta,
        int& maxWinX,
        int& maxWinY,
        int& maxWinTheta,
        double& maxScore) const;

    /* Find a corresponding pose of the current robot pose
     * from the loop-closure candidate local grid map */
    bool FindCorrespondingPose(
        const GridMapBuilder::LocalMapInfo& localMapInfo,
        const Sensor::ScanDataPtr<double>& scanData,
        const RobotPose2D<double>& robotPose,
        RobotPose2D<double>& correspondingPose,
        Eigen::Matrix3d& estimatedCovMat) const;

private:
    /* Cost function */
    CostFuncPtr                 mCostFunc;
    /* Loop closure candidate search */
    LoopClosureCandidateNearest mLoopClosureCandidate;
    /* Resolution for low resolution map (in the number of grid cells) */
    int                         mLowResolution;
    /* Linear (horizontal) size of the searching window */
    double                      mRangeX;
    /* Linear (vertical) size of the searching window */
    double                      mRangeY;
    /* Angular size of the search window */
    double                      mRangeTheta;
    /* Maximum laser scan range considered for loop closure */
    double                      mScanRangeMax;
    /* Normalized matching score threshold for loop closure */
    double                      mScoreThreshold;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_CLOSURE_REAL_TIME_CORRELATIVE_HPP */
