
/* loop_closure_grid_search.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_CLOSURE_GRID_SEARCH_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_CLOSURE_GRID_SEARCH_HPP

#include "my_lidar_graph_slam/mapping/cost_function.hpp"
#include "my_lidar_graph_slam/mapping/score_function.hpp"
#include "my_lidar_graph_slam/mapping/loop_closure.hpp"
#include "my_lidar_graph_slam/mapping/loop_closure_candidate_nearest.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

class LoopClosureGridSearch final : public LoopClosure
{
public:
    /* Type definitions */
    using LoopClosure::GridMapBuilderPtr;
    using LoopClosure::PoseGraphPtr;
    using LoopClosure::ScanPtr;
    using LoopClosure::GridMapType;

    using CostFuncPtr = std::shared_ptr<CostFunction<GridMapType, ScanPtr>>;
    using ScoreFuncPtr = std::shared_ptr<ScoreFunction>;

    /* Constructor */
    LoopClosureGridSearch(const ScoreFuncPtr& scoreFunc,
                          const CostFuncPtr& costFunc,
                          double travelDistThreshold,
                          double nodeDistThreshold,
                          double rangeX,
                          double rangeY,
                          double rangeTheta,
                          double stepX,
                          double stepY,
                          double stepTheta,
                          double scoreThreshold,
                          double matchRateThreshold) :
        mScoreFunc(scoreFunc),
        mCostFunc(costFunc),
        mLoopClosureCandidate(travelDistThreshold, nodeDistThreshold),
        mRangeX(rangeX),
        mRangeY(rangeY),
        mRangeTheta(rangeTheta),
        mStepX(stepX),
        mStepY(stepY),
        mStepTheta(stepTheta),
        mScoreThreshold(scoreThreshold),
        mMatchRateThreshold(matchRateThreshold) { }

    /* Destructor */
    ~LoopClosureGridSearch() = default;

    /* Find a loop and return a loop constraint */
    bool FindLoop(const GridMapBuilderPtr& gridMapBuilder,
                  const PoseGraphPtr& poseGraph,
                  RobotPose2D<double>& relPose,
                  int& startNodeIdx,
                  int& endNodeIdx,
                  Eigen::Matrix3d& estimatedCovMat) override;
    
private:
    /* Find a corresponding pose of the current robot pose
     * from the loop-closure candidate local grid map */
    bool FindCorrespondingPose(const GridMapType& gridMap,
                               const ScanPtr& scanData,
                               const RobotPose2D<double>& robotPose,
                               RobotPose2D<double>& correspondingPose,
                               Eigen::Matrix3d& estimatedCovMat) const;

private:
    /* Matching score function */
    ScoreFuncPtr                mScoreFunc;
    /* Cost function */
    CostFuncPtr                 mCostFunc;
    /* Loop closure candidate search */
    LoopClosureCandidateNearest mLoopClosureCandidate;
    /* Linear (horizontal) size of the search window */
    double                      mRangeX;
    /* Linear (vertical) size of the search window */
    double                      mRangeY;
    /* Angular size of the search window */
    double                      mRangeTheta;
    /* Linear (horizontal) step size */
    double                      mStepX;
    /* Linear (vertical) step size */
    double                      mStepY;
    /* Angular step size */
    double                      mStepTheta;
    /* Normalized matching score threshold for loop closure */
    double                      mScoreThreshold;
    /* Match rate threshold for loop closure */
    double                      mMatchRateThreshold;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_CLOSURE_GRID_SEARCH_HPP */
