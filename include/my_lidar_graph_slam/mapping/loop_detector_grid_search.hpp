
/* loop_detector_grid_search.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_DETECTOR_GRID_SEARCH_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_DETECTOR_GRID_SEARCH_HPP

#include "my_lidar_graph_slam/mapping/loop_detector.hpp"

#include <memory>

#include "my_lidar_graph_slam/util.hpp"
#include "my_lidar_graph_slam/mapping/scan_matcher_grid_search.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

class LoopDetectorGridSearch final : public LoopDetector
{
public:
    /* Constructor */
    LoopDetectorGridSearch(
        const std::shared_ptr<ScanMatcherGridSearch>& scanMatcher,
        const double scoreThreshold,
        const double knownRateThreshold);

    /* Destructor */
    ~LoopDetectorGridSearch() = default;

    /* Find a loop and return a loop constraint */
    void Detect(
        LoopDetectionQueryVector& loopDetectionQueries,
        LoopDetectionResultVector& loopDetectionResults) override;

private:
    /* Find a corresponding pose of the current robot pose
     * from the local grid map */
    bool FindCorrespondingPose(const GridMapType& gridMap,
                               const Sensor::ScanDataPtr<double>& scanData,
                               const RobotPose2D<double>& mapLocalScanPose,
                               RobotPose2D<double>& correspondingPose,
                               Eigen::Matrix3d& estimatedCovMat) const;

private:
    /* Exhaustive grid search based scan matcher */
    std::shared_ptr<ScanMatcherGridSearch> mScanMatcher;
    /* Normalized matching score threshold for loop detector */
    const double                           mScoreThreshold;
    /* Threshold for the ratio of the known grid cells */
    const double                           mKnownRateThreshold;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_DETECTOR_GRID_SEARCH_HPP */
