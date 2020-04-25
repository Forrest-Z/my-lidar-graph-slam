
/* loop_closure_branch_bound.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_CLOSURE_BRANCH_BOUND_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_CLOSURE_BRANCH_BOUND_HPP

#include <stack>

#include "my_lidar_graph_slam/util.hpp"
#include "my_lidar_graph_slam/mapping/cost_function.hpp"
#include "my_lidar_graph_slam/mapping/score_function.hpp"
#include "my_lidar_graph_slam/mapping/loop_closure.hpp"
#include "my_lidar_graph_slam/mapping/loop_closure_candidate_nearest.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

class LoopClosureBranchBound final : public LoopClosure
{
public:
    /* Type definitions */
    using LoopClosure::GridMapBuilderPtr;
    using LoopClosure::PoseGraphPtr;
    using LoopClosure::ScanPtr;
    using LoopClosure::GridMapType;
    using LoopClosure::PrecomputedMapType;

    using CostFunctionType = CostFunction<GridMapType, ScanPtr>;
    using CostFuncPtr = std::shared_ptr<CostFunctionType>;
    using ScoreFunctionType = ScoreFunction<PrecomputedMapType, ScanPtr>;
    using ScoreFuncPtr = std::shared_ptr<ScoreFunctionType>;

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
                           double travelDistThreshold,
                           double nodeDistThreshold,
                           double scoreThreshold,
                           double matchRateThreshold) :
        mScoreFunc(scoreFunc),
        mCostFunc(costFunc),
        mLoopClosureCandidate(travelDistThreshold, nodeDistThreshold),
        mScoreThreshold(scoreThreshold),
        mMatchRateThreshold(matchRateThreshold) { }

    /* Destructor */
    ~LoopClosureBranchBound() = default;

    /* Find a loop and return a loop constraint */
    bool FindLoop(GridMapBuilderPtr& gridMapBuilder,
                  const PoseGraphPtr& poseGraph,
                  RobotPose2D<double>& relPose,
                  int& startNodeIdx,
                  int& endNodeIdx,
                  Eigen::Matrix3d& estimatedCovMat) override;

private:
    /* Precompute grid maps for efficient loop closure */
    void PrecomputeGridMaps(GridMapBuilderPtr& gridMapBuilder,
                            int mapIdx) const;

    /* Compute the maximum of a 2^h pixel wide row starting at each pixel */
    void SlidingWindowMaxRow(const GridMapType& gridMap,
                             PrecomputedMapType& tmpMap,
                             int winSize) const;

    /* Compute the maximum of a 2^h pixel wide column starting at each pixel */
    void SlidingWindowMaxCol(const PrecomputedMapType& tmpMap,
                             PrecomputedMapType& precompMap,
                             int winSize) const;

    /* Find a corresponding pose of the current robot pose
     * from the loop-closure candidate local grid map */
    bool FindCorrespondingPose(
        const GridMapBuilder::LocalMapInfo& localMapInfo,
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
    /* Normalized matching score threshold for loop closure */
    double                      mScoreThreshold;
    /* Match rate threshold for loop closure */
    double                      mMatchRateThreshold;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_CLOSURE_BRANCH_BOUND_HPP */
