
/* pose_graph_optimizer_lm.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_POSE_GRAPH_OPTIMIZER_LM_HPP
#define MY_LIDAR_GRAPH_SLAM_POSE_GRAPH_OPTIMIZER_LM_HPP

#include "my_lidar_graph_slam/mapping/pose_graph_optimizer.hpp"

#include <cmath>
#include <limits>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/IterativeLinearSolvers>

#include "my_lidar_graph_slam/pose.hpp"
#include "my_lidar_graph_slam/util.hpp"
#include "my_lidar_graph_slam/mapping/robust_loss_function.hpp"

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
    /*
     * SolverType enum specifies the linear solver used in this class
     */
    enum class SolverType
    {
        SparseCholesky,
        ConjugateGradient,
    };

public:
    /* Constructor */
    PoseGraphOptimizerLM(const SolverType solverType,
                         const int numOfIterationsMax,
                         const double errorTolerance,
                         const double initialLambda,
                         const LossFunctionPtr& lossFunction) :
        mSolverType(solverType),
        mNumOfIterationsMax(numOfIterationsMax),
        mErrorTolerance(errorTolerance),
        mLambda(initialLambda),
        mLossFunction(lossFunction) { }
    /* Destructor */
    ~PoseGraphOptimizerLM() = default;

    /* Optimize a pose graph using the combination of
     * linear solver and Levenberg-Marquardt method */
    void Optimize(
        IdMap<LocalMapId, LocalMapNode>& localMapNodes,
        IdMap<NodeId, ScanNode>& scanNodes,
        const std::vector<PoseGraphEdge>& poseGraphEdges) override;

    /* Compute error function */
    void ComputeErrorFunction(
        const RobotPose2D<double>& startNodePose,
        const RobotPose2D<double>& endNodePose,
        const RobotPose2D<double>& edgeRelPose,
        Eigen::Vector3d& errorVec) const override;

private:
    /* Perform one optimization step and return the total error */
    void OptimizeStep(
        IdMap<LocalMapId, LocalMapNode>& localMapNodes,
        IdMap<NodeId, ScanNode>& scanNodes,
        const std::vector<PoseGraphEdge>& poseGraphEdges,
        Eigen::SparseMatrix<double>& matA,
        Eigen::VectorXd& vecB,
        Eigen::VectorXd& vecDelta,
        std::vector<Eigen::Triplet<double>>& matATriplets);

    /* Compute Jacobian matrices of the error function with respect to the
     * starting pose and ending pose of the pose graph edge */
    void ComputeErrorJacobians(
        const RobotPose2D<double>& startNodePose,
        const RobotPose2D<double>& endNodePose,
        Eigen::Matrix3d& startNodeErrorJacobian,
        Eigen::Matrix3d& endNodeErrorJacobian) const;

    /* Compute total error */
    double ComputeTotalError(
        const IdMap<LocalMapId, LocalMapNode>& localMapNodes,
        const IdMap<NodeId, ScanNode>& scanNodes,
        const std::vector<PoseGraphEdge>& poseGraphEdges) const;

    /* Dump the pose graph error */
    void DumpError(
        const IdMap<LocalMapId, LocalMapNode>& localMapNodes,
        const IdMap<NodeId, ScanNode>& scanNodes,
        const std::vector<PoseGraphEdge>& poseGraphEdges) const;

private:
    /* Linear solver type */
    const SolverType                    mSolverType;
    /* Maximum number of the optimization iterations */
    const int                           mNumOfIterationsMax;
    /* Error tolerance to check the convergence */
    const double                        mErrorTolerance;
    /* Damping factor used in Levenberg-Marquardt method
     * The method is almost the same as Gauss-Newton method when small,
     * and is gradient descent method when large */
    double                              mLambda;
    /* Robust loss function to correct (weight) information matrices */
    LossFunctionPtr                     mLossFunction;

    /* Left-hand side sparse matrix of the linear system */
    Eigen::SparseMatrix<double>         mMatA;
    /* Right-hand side vector of the linear system */
    Eigen::VectorXd                     mVecB;
    /* Result of the Sparse Cholesky factorization */
    Eigen::VectorXd                     mVecDelta;
    /* Vector of the sparse matrix non-zero elements (triplets) */
    std::vector<Eigen::Triplet<double>> mMatATriplets;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_POSE_GRAPH_OPTIMIZER_LM_HPP */
