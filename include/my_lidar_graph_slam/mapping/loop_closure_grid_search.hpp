
/* loop_closure_grid_search.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_CLOSURE_GRID_SEARCH_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_CLOSURE_GRID_SEARCH_HPP

#include "my_lidar_graph_slam/mapping/cost_function.hpp"
#include "my_lidar_graph_slam/mapping/loop_closure.hpp"

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

    /* Constructor */
    LoopClosureGridSearch(const CostFuncPtr& costFunc,
                          double travelDistThreshold,
                          double poseGraphNodeDistMax,
                          double rangeX,
                          double rangeY,
                          double rangeTheta,
                          double stepX,
                          double stepY,
                          double stepTheta,
                          double costThreshold) :
        mCostFunc(costFunc),
        mTravelDistThreshold(travelDistThreshold),
        mPoseGraphNodeDistMax(poseGraphNodeDistMax),
        mRangeX(rangeX),
        mRangeY(rangeY),
        mRangeTheta(rangeTheta),
        mStepX(stepX),
        mStepY(stepY),
        mStepTheta(stepTheta),
        mCostThreshold(costThreshold) { }

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
    /* Find a local map and a pose graph node as loop closure candidates */
    bool FindLoopClosureCandidates(const GridMapBuilderPtr& gridMapBuilder,
                                   const PoseGraphPtr& poseGraph,
                                   const RobotPose2D<double>& robotPose,
                                   int& candidateMapIdx,
                                   int& candidateNodeIdx) const;

    /* Find a corresponding pose of the current robot pose
     * from the loop-closure candidate local grid map */
    bool FindCorrespondingPose(const GridMapType& gridMap,
                               const ScanPtr& scanData,
                               const RobotPose2D<double>& robotPose,
                               RobotPose2D<double>& correspondingPose,
                               Eigen::Matrix3d& estimatedCovMat) const;

private:
    /* Cost function */
    CostFuncPtr    mCostFunc;
    /* Minimum travel distance difference for loop closure */
    double         mTravelDistThreshold;
    /* Maximum distance between the current robot pose and pose of the
     * loop-closure candidate pose graph node */
    double         mPoseGraphNodeDistMax;
    /* Linear (horizontal) size of the search window */
    double         mRangeX;
    /* Linear (vertical) size of the search window */
    double         mRangeY;
    /* Angular size of the search window */
    double         mRangeTheta;
    /* Linear (horizontal) step size */
    double         mStepX;
    /* Linear (vertical) step size */
    double         mStepY;
    /* Angular step size */
    double         mStepTheta;
    /* Normalized cost threshold for loop closure */
    double         mCostThreshold;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_CLOSURE_GRID_SEARCH_HPP */
