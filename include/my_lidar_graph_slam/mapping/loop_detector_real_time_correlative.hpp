
/* loop_detector_real_time_correlative.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_DETECTOR_REAL_TIME_CORRELATIVE_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_DETECTOR_REAL_TIME_CORRELATIVE_HPP

#include <memory>

#include "my_lidar_graph_slam/mapping/cost_function.hpp"
#include "my_lidar_graph_slam/mapping/scan_matcher_real_time_correlative.hpp"
#include "my_lidar_graph_slam/mapping/score_function.hpp"
#include "my_lidar_graph_slam/mapping/loop_detector.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

class LoopDetectorRealTimeCorrelative final : public LoopDetector
{
public:
    /* Constructor */
    LoopDetectorRealTimeCorrelative(
        const std::shared_ptr<ScanMatcherRealTimeCorrelative>& scanMatcher,
        const double scoreThreshold);

    /* Destructor */
    ~LoopDetectorRealTimeCorrelative() = default;

    /* Find a loop and return a loop constraint */
    void Detect(
        LoopDetectionQueryVector& loopDetectionQueries,
        LoopDetectionResultVector& loopDetectionResults) override;

private:
    /* Find a corresponding pose of the current robot pose
     * from a local grid map */
    bool FindCorrespondingPose(
        const GridMapType& localMap,
        const std::map<int, PrecomputedMapType>& precompMaps,
        const Sensor::ScanDataPtr<double>& scanData,
        const RobotPose2D<double>& robotPose,
        RobotPose2D<double>& correspondingPose,
        Eigen::Matrix3d& estimatedCovMat) const;

private:
    /* Real-time correlative scan matcher */
    std::shared_ptr<ScanMatcherRealTimeCorrelative> mScanMatcher;
    /* Normalized matching score threshold for loop detection */
    const double                                    mScoreThreshold;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_DETECTOR_REAL_TIME_CORRELATIVE_HPP */
