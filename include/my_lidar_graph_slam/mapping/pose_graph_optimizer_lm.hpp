
/* pose_graph_optimizer_lm.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_POSE_GRAPH_OPTIMIZER_LM_HPP
#define MY_LIDAR_GRAPH_SLAM_POSE_GRAPH_OPTIMIZER_LM_HPP

#include <cmath>
#include <limits>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "my_lidar_graph_slam/pose.hpp"
#include "my_lidar_graph_slam/util.hpp"
#include "my_lidar_graph_slam/mapping/pose_graph_optimizer.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/*
 * PoseGraphOptimizerLM class optimizes the pose graph using the
 * combination of linear solver and Levenberg-Marquardt method
 * which is also called Sparse Pose Adjustment (SPA) as proposed in the
 * following paper:
 * K. Konolige, G. Grisetti, R. Kuemmerle, W. Burgard, B. Limketkai, and
 * R. Vincent. "Efficient Sparse Pose Adjustment for 2D Mapping," in the
 * Proceedings of the IEEE/RSJ International Conference on Intelligent Robots
 * and Systems (IROS), 2010.
 */
class PoseGraphOptimizerLM final : public PoseGraphOptimizer
{
public:
    /* Constructor */
    PoseGraphOptimizerLM(int numOfIterationsMax,
                         double errorTolerance,
                         double initialLambda) :
        mNumOfIterationsMax(numOfIterationsMax),
        mErrorTolerance(errorTolerance),
        mLambda(initialLambda) { }
    /* Destructor */
    ~PoseGraphOptimizerLM() = default;

    /* Optimize a pose graph using the combination of
     * linear solver and Levenberg-Marquardt method */
    void Optimize(std::shared_ptr<PoseGraph>& poseGraph) override;

private:
    /* Perform one optimization step and return the total error */
    void OptimizeStep(std::shared_ptr<PoseGraph>& poseGraph,
                      Eigen::SparseMatrix<double>& matA,
                      Eigen::VectorXd& vecB,
                      Eigen::VectorXd& vecDelta,
                      std::vector<Eigen::Triplet<double>>& matATriplets);

    /* Compute Jacobian matrices of the error function with respect to the
     * starting pose and ending pose of the pose graph edge */
    void ComputeErrorJacobians(const RobotPose2D<double>& startNodePose,
                               const RobotPose2D<double>& endNodePose,
                               Eigen::Matrix3d& startNodeErrorJacobian,
                               Eigen::Matrix3d& endNodeErrorJacobian) const;
    
    /* Compute error function */
    void ComputeErrorFunction(const RobotPose2D<double>& startNodePose,
                              const RobotPose2D<double>& endNodePose,
                              const RobotPose2D<double>& edgeRelPose,
                              Eigen::Vector3d& errorVec) const;
    
    /* Compute total error */
    double ComputeTotalError(const std::shared_ptr<PoseGraph>& poseGraph) const;

private:
    /* Maximum number of the optimization iterations */
    int    mNumOfIterationsMax;
    /* Error tolerance to check the convergence */
    double mErrorTolerance;
    /* Damping factor used in Levenberg-Marquardt method
     * The method is almost the same as Gauss-Newton method when small,
     * and is gradient descent method when large */
    double mLambda;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_POSE_GRAPH_OPTIMIZER_LM_HPP */
