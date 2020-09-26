
/* loop_closure_grid_search.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_CLOSURE_GRID_SEARCH_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_CLOSURE_GRID_SEARCH_HPP

#include "my_lidar_graph_slam/util.hpp"
#include "my_lidar_graph_slam/mapping/cost_function.hpp"
#include "my_lidar_graph_slam/mapping/score_function.hpp"
#include "my_lidar_graph_slam/mapping/loop_closure.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

class LoopClosureGridSearch final : public LoopClosure
{
public:
    /* Type definitions */
    using LoopClosure::ScanPtr;
    using LoopClosure::GridMapType;

    /* Constructor */
    LoopClosureGridSearch(const ScoreFuncPtr& scoreFunc,
                          const CostFuncPtr& costFunc,
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
    bool FindLoop(
        LoopClosureCandidateInfoVector& loopClosureCandidates,
        LoopClosureResultVector& loopClosureResults) override;

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
