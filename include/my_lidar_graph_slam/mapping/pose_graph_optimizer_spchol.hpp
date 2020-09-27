
/* pose_graph_optimizer_spchol.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_POSE_GRAPH_OPTIMIZER_SPCHOL_HPP
#define MY_LIDAR_GRAPH_SLAM_POSE_GRAPH_OPTIMIZER_SPCHOL_HPP

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
 * PoseGraphOptimizerSpChol class optimizes the pose graph using the
 * combination of Sparse Cholesky Factorization and Levenberg-Marquardt method
 * which is also called Sparse Pose Adjustment (SPA) as proposed in the
 * following paper:
 * K. Konolige, G. Grisetti, R. Kuemmerle, W. Burgard, B. Limketkai, and
 * R. Vincent. "Efficient Sparse Pose Adjustment for 2D Mapping," in the
 * Proceedings of the IEEE/RSJ International Conference on Intelligent Robots
 * and Systems (IROS), 2010.
 */
class PoseGraphOptimizerSpChol final : public PoseGraphOptimizer
{
public:
    /* Constructor */
    PoseGraphOptimizerSpChol(int numOfIterationsMax,
                             double errorTolerance,
                             double initialLambda) :
        mNumOfIterationsMax(numOfIterationsMax),
        mErrorTolerance(errorTolerance),
        mLambda(initialLambda) { }
    /* Destructor */
    ~PoseGraphOptimizerSpChol() = default;

    /* Optimize a pose graph using the combination of
     * Sparse Cholesky Factorization and Levenberg-Marquardt method */
    void Optimize(
        std::vector<PoseGraph::Node>& poseGraphNodes,
        const std::vector<PoseGraph::Edge>& poseGraphEdges) override;

private:
    /* Perform one optimization step and return the total error */
    void OptimizeStep(std::vector<PoseGraph::Node>& poseGraphNodes,
                      const std::vector<PoseGraph::Edge>& poseGraphEdges,
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
    double ComputeTotalError(
        const std::vector<PoseGraph::Node>& poseGraphNodes,
        const std::vector<PoseGraph::Edge>& poseGraphEdges) const;

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

#endif /* MY_LIDAR_GRAPH_SLAM_POSE_GRAPH_OPTIMIZER_SPCHOL_HPP */
