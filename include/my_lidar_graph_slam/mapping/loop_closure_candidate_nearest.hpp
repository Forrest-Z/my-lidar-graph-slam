
/* loop_closure_candidate_nearest.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_CLOSURE_CANDIDATE_NEAREST_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_CLOSURE_CANDIDATE_NEAREST_HPP

#include "my_lidar_graph_slam/mapping/loop_closure_candidate.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

class LoopClosureCandidateNearest final : public LoopClosureCandidate
{
public:
    /* Type definitions */
    using LoopClosureCandidate::GridMapBuilderPtr;
    using LoopClosureCandidate::PoseGraphPtr;
    using LoopClosureCandidate::ScanPtr;
    using LoopClosureCandidate::GridMapType;

    using LoopClosureCandidate::CandidateType;
    using LoopClosureCandidate::CandidateVector;
    
    /* Constructor */
    LoopClosureCandidateNearest(double travelDistThreshold,
                                double nodeDistThreshold) :
        LoopClosureCandidate(),
        mTravelDistThreshold(travelDistThreshold),
        mNodeDistThreshold(nodeDistThreshold) { }

    /* Destructor */
    ~LoopClosureCandidateNearest() = default;

    /* Find a local map and a pose graph node used for loop closure */
    CandidateVector Find(const GridMapBuilderPtr& gridMapBuilder,
                         const PoseGraphPtr& poseGraph,
                         const RobotPose2D<double>& robotPose) override;

private:
    /* Travel distance threhsold for loop closure
     * Pose graph node that can be traversed from the current node
     * with the travel distance less than this threshold is not considered
     * for loop closure */
    double mTravelDistThreshold;
    /* Maximum distance between the current robot pose and
     * the pose of the node which is considered for loop closure */
    double mNodeDistThreshold;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_LOOP_CLOSURE_CANDIDATE_NEAREST_HPP */
