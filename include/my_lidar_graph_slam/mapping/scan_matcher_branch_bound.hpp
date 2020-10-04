
/* scan_matcher_branch_bound.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_SCAN_MATCHER_BRANCH_BOUND_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_SCAN_MATCHER_BRANCH_BOUND_HPP

#include "my_lidar_graph_slam/mapping/scan_matcher.hpp"

#include <map>
#include <memory>
#include <stack>

#include "my_lidar_graph_slam/mapping/cost_function.hpp"
#include "my_lidar_graph_slam/mapping/grid_map_builder.hpp"
#include "my_lidar_graph_slam/mapping/score_function_pixel_accurate.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

class ScanMatcherBranchBound final : public ScanMatcher
{
private:
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
    ScanMatcherBranchBound(
        const std::shared_ptr<ScorePixelAccurate>& scoreFunc,
        const CostFuncPtr& costFunc,
        const int nodeHeightMax,
        const double rangeX,
        const double rangeY,
        const double rangeTheta,
        const double scanRangeMax);

    /* Destructor */
    ~ScanMatcherBranchBound() = default;

    /* Optimize the robot pose by scan matching */
    ScanMatchingSummary OptimizePose(
        const ScanMatchingQuery& queryInfo) override;

    /* Optimize the robot pose by scan matching */
    ScanMatchingSummary OptimizePose(
        const GridMapType& gridMap,
        const std::map<int, ConstMapType>& precompMaps,
        const Sensor::ScanDataPtr<double>& scanData,
        const RobotPose2D<double>& mapLocalInitialPose,
        const double normalizedScoreThreshold) const;

    /* Precompute multiple coarser grid maps for scan matching */
    std::map<int, ConstMapType> ComputeCoarserMaps(
        const GridMapType& gridMap) const;

private:
    /* Compute the search window step */
    void ComputeSearchStep(
        const GridMapType& gridMap,
        const Sensor::ScanDataPtr<double>& scanData,
        double& stepX,
        double& stepY,
        double& stepTheta) const;

private:
    /* Pixel-accurate score function */
    std::shared_ptr<ScorePixelAccurate> mScoreFunc;
    /* Cost function */
    CostFuncPtr                         mCostFunc;
    /* Maximum height of the tree used in branch-and-bound */
    const int                           mNodeHeightMax;
    /* Linear (horizontal) size of the searching window */
    const double                        mRangeX;
    /* Linear (vertical) size of the searching window */
    const double                        mRangeY;
    /* Angular size of the searching window */
    const double                        mRangeTheta;
    /* Maximum laser scan range considered for loop detector */
    const double                        mScanRangeMax;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_SCAN_MATCHER_BRANCH_BOUND_HPP */
