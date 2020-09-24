
/* loop_closure_branch_bound.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_CLOSURE_BRANCH_BOUND_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_CLOSURE_BRANCH_BOUND_HPP

#include <map>
#include <stack>

#include "my_lidar_graph_slam/util.hpp"
#include "my_lidar_graph_slam/mapping/cost_function.hpp"
#include "my_lidar_graph_slam/mapping/score_function.hpp"
#include "my_lidar_graph_slam/mapping/loop_closure.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

class LoopClosureBranchBound final : public LoopClosure
{
public:
    /* Type definitions */
    using LoopClosure::ScanPtr;
    using LoopClosure::GridMapType;
    using LoopClosure::PrecomputedMapType;

    /*
     * Node struct holds the necessary information for Branch-and-Bound method
     */
    struct Node
    {
        /* Constructor */
        Node(int x, int y, int theta, int height) :
            mX(x), mY(y), mTheta(theta), mHeight(height) { }

        /* Check if the node is a leaf node */
        inline bool IsLeafNode() const { return this->mHeight == 0; }

        int mX;
        int mY;
        int mTheta;
        int mHeight;
    };

public:
    /* Constructor */
    LoopClosureBranchBound(const ScoreFuncPtr& scoreFunc,
                           const CostFuncPtr& costFunc,
                           int nodeHeightMax,
                           double rangeX,
                           double rangeY,
                           double rangeTheta,
                           double scanRangeMax,
                           double scoreThreshold,
                           double matchRateThreshold) :
        mScoreFunc(scoreFunc),
        mCostFunc(costFunc),
        mNodeHeightMax(nodeHeightMax),
        mRangeX(rangeX),
        mRangeY(rangeY),
        mRangeTheta(rangeTheta),
        mScanRangeMax(scanRangeMax),
        mScoreThreshold(scoreThreshold),
        mMatchRateThreshold(matchRateThreshold) { }

    /* Destructor */
    ~LoopClosureBranchBound() = default;

    /* Find a loop and return a loop constraint */
    bool FindLoop(
        LoopClosureCandidateInfoVector& loopClosureCandidates,
        LoopClosureResultVector& loopClosureResults) override;

private:
    /* Find a corresponding pose of the current robot pose
     * from the loop-closure candidate local grid map */
    bool FindCorrespondingPose(
        const GridMapType& localMap,
        const std::map<int, PrecomputedMapType>& precompMaps,
        const ScanPtr& scanData,
        const RobotPose2D<double>& robotPose,
        RobotPose2D<double>& correspondingPose,
        Eigen::Matrix3d& estimatedCovMat) const;

private:
    /* Matching score function */
    ScoreFuncPtr                mScoreFunc;
    /* Cost function */
    CostFuncPtr                 mCostFunc;
    /* Maximum height of the tree used in branch-and-bound */
    int                         mNodeHeightMax;
    /* Linear (horizontal) size of the search window */
    double                      mRangeX;
    /* Linear (vertical) size of the search window */
    double                      mRangeY;
    /* Angular size of the search window */
    double                      mRangeTheta;
    /* Maximum laser scan range considered for loop closure */
    double                      mScanRangeMax;
    /* Normalized matching score threshold for loop closure */
    double                      mScoreThreshold;
    /* Match rate threshold for loop closure */
    double                      mMatchRateThreshold;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_CLOSURE_BRANCH_BOUND_HPP */
