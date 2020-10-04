
/* loop_detector_branch_bound.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_DETECTOR_BRANCH_BOUND_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_DETECTOR_BRANCH_BOUND_HPP

#include <map>
#include <stack>

#include "my_lidar_graph_slam/util.hpp"
#include "my_lidar_graph_slam/mapping/cost_function.hpp"
#include "my_lidar_graph_slam/mapping/scan_matcher_branch_bound.hpp"
#include "my_lidar_graph_slam/mapping/score_function.hpp"
#include "my_lidar_graph_slam/mapping/loop_detector.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

class LoopDetectorBranchBound final : public LoopDetector
{
public:
    /* Constructor */
    LoopDetectorBranchBound(
        const std::shared_ptr<ScanMatcherBranchBound>& scanMatcher,
        const double scoreThreshold);

    /* Destructor */
    ~LoopDetectorBranchBound() = default;

    /* Find a loop and return a loop constraint */
    void Detect(
        LoopDetectionQueryVector& loopDetectionQueries,
        LoopDetectionResultVector& loopDetectionResults) override;

private:
    /* Find a corresponding pose of the current robot pose
     * from the local grid map */
    bool FindCorrespondingPose(
        const GridMapType& localMap,
        const std::map<int, ConstMapType>& precompMaps,
        const Sensor::ScanDataPtr<double>& scanData,
        const RobotPose2D<double>& mapLocalScanPose,
        RobotPose2D<double>& correspondingPose,
        Eigen::Matrix3d& estimatedCovMat) const;

private:
    /* Branch-and-bound based scan matcher */
    std::shared_ptr<ScanMatcherBranchBound> mScanMatcher;
    /* Normalized matching score threshold for loop detector */
    const double                            mScoreThreshold;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_DETECTOR_BRANCH_BOUND_HPP */
