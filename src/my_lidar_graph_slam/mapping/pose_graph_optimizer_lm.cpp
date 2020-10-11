
/* pose_graph_optimizer_lm.cpp */

#include "my_lidar_graph_slam/mapping/pose_graph_optimizer_lm.hpp"

#include "my_lidar_graph_slam/metric/metric.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/* Optimize a pose graph using the combination of
 * linear solver and Levenberg-Marquardt method */
void PoseGraphOptimizerLM::Optimize(
    IdMap<LocalMapId, LocalMapNode>& localMapNodes,
    IdMap<NodeId, ScanNode>& scanNodes,
    const std::vector<PoseGraphEdge>& poseGraphEdges)
{
    double prevTotalError = std::numeric_limits<double>::max();
    double totalError = std::numeric_limits<double>::max();
    int numOfIterations = 0;

    /* Retrieve the number of nodes and edges
     * These remain constant during the optimization */
    const int numOfLocalMapNodes = static_cast<int>(localMapNodes.size());
    const int numOfScanNodes = static_cast<int>(scanNodes.size());
    const int numOfEdges = static_cast<int>(poseGraphEdges.size());

    /* Total number of the variables in the linear system */
    const int numOfVariables = 3 * (numOfLocalMapNodes + numOfScanNodes);
    /* Total number of the non-zero elements in the sparse matrix */
    const std::size_t numOfNonzeroElements = 4 * (3 * 3) * numOfEdges;

    /* Resize vectors and matrices */
    /* Resize the left-hand side sparse matrix of the linear system */
    this->mMatA.resize(numOfVariables, numOfVariables);
    /* Resize the right-hand side vector of the linear system */
    this->mVecB.resize(numOfVariables);
    /* Resize the result of the Sparse Cholesky factorization */
    this->mVecDelta.resize(numOfVariables);
    /* Resize the vector of the sparse matrix non-zero elements */
    this->mMatATriplets.reserve(numOfNonzeroElements);

    while (true) {
        /* Perform one optimization step */
        this->OptimizeStep(localMapNodes, scanNodes, poseGraphEdges,
                           this->mMatA, this->mVecB,
                           this->mVecDelta, this->mMatATriplets);
        /* Compute the total error */
        totalError = this->ComputeTotalError(
            localMapNodes, scanNodes, poseGraphEdges);

        /* Stop the graph optimization if the number of iteration steps
         * exceeded the maximum or the total error converged */
        if (++numOfIterations >= this->mNumOfIterationsMax ||
            std::fabs(prevTotalError - totalError) < this->mErrorTolerance)
            break;

        /* Update damping factor (lambda)
         * If error decreased, halve the damping factor
         * If error increased, double the damping factor */
        if (totalError < prevTotalError)
            this->mLambda *= 0.5;
        else
            this->mLambda *= 2.0;

        prevTotalError = totalError;
    }
}

/* Perform one optimization step and return the total error */
void PoseGraphOptimizerLM::OptimizeStep(
    IdMap<LocalMapId, LocalMapNode>& localMapNodes,
    IdMap<NodeId, ScanNode>& scanNodes,
    const std::vector<PoseGraphEdge>& poseGraphEdges,
    Eigen::SparseMatrix<double>& matA,
    Eigen::VectorXd& vecB,
    Eigen::VectorXd& vecDelta,
    std::vector<Eigen::Triplet<double>>& matATriplets)
{
    const int numOfLocalMapNodes = static_cast<int>(localMapNodes.size());
    const int numOfScanNodes = static_cast<int>(scanNodes.size());
    const int numOfVariables = 3 * (numOfLocalMapNodes + numOfScanNodes);

    /* Clear all vectors and matrices */
    matATriplets.clear();
    vecB.setZero();

    /* Setup the left-hand side sparse matrix H */
    for (const auto& edge : poseGraphEdges) {
        /* Retrieve the relative pose \bar{z}_{ij} from the edge */
        const RobotPose2D<double>& edgeRelPose = edge.mRelativePose;
        /* Retrieve the information matrix \Lambda_{ij} of the edge */
        const Eigen::Matrix3d& infoMat = edge.mInformationMat;

        /* Retrieve the poses of the start node and the end node */
        const LocalMapId startNodeId = edge.mLocalMapNodeId;
        const NodeId endNodeId = edge.mScanNodeId;

        /* Get the iterators to the node */
        const auto startNodeIt = localMapNodes.find(startNodeId);
        const auto endNodeIt = scanNodes.find(endNodeId);

        /* Compute the indices to the matrix elements */
        const int offsetToStartNode =
            std::distance(localMapNodes.begin(), startNodeIt);
        const int offsetToEndNode =
            std::distance(scanNodes.begin(), endNodeIt);
        const int startNodeIdx = offsetToStartNode;
        const int endNodeIdx = numOfLocalMapNodes + offsetToEndNode;

        const RobotPose2D<double>& startNodePose =
            localMapNodes.at(startNodeId).mGlobalPose;
        const RobotPose2D<double>& endNodePose =
            scanNodes.at(endNodeId).mGlobalPose;

        /* Compute Jacobian matrices of the error function */
        Eigen::Matrix3d startNodeJacobian;
        Eigen::Matrix3d endNodeJacobian;
        this->ComputeErrorJacobians(startNodePose, endNodePose,
                                    startNodeJacobian, endNodeJacobian);

        /* Compute error function */
        Eigen::Vector3d errorVec;
        this->ComputeErrorFunction(startNodePose, endNodePose,
                                   edgeRelPose, errorVec);

        /* Correct the information matrix using the weight function
         * based on the robust estimation (M-estimation)
         * to prevent outliers caused by wrong loop detections */
        const double errorWeight = edge.IsLoopClosingConstraint() ?
            this->mLossFunction->Weight(
                errorVec.transpose() * infoMat * errorVec) : 1.0;
        const Eigen::Matrix3d weightedInfoMat = errorWeight * infoMat;

        /* Compute 4 non-zero block matrices denoted as 
         * J_i^T \Lambda_{ij} J_i, J_i^T \Lambda_{ij} J_j,
         * J_j^T \Lambda_{ij} J_i, and J_j^T \Lambda_{ij} J_j in the paper
         * Note that J_j^T \Lambda_{ij} J_i is the transpose of
         * J_i^T \Lambda_{ij} J_j and thus is not calculated */

        /* Compute the intermediate results, namely
         * J_i^T \Lambda_{ij} and J_j^T \Lambda_{ij} */
        const Eigen::Matrix3d trJsInfo =
            startNodeJacobian.transpose() * weightedInfoMat;
        const Eigen::Matrix3d trJeInfo =
            endNodeJacobian.transpose() * weightedInfoMat;

        /* Compute 4 non-zero block matrices */
        const Eigen::Matrix3d trJsInfoJs = trJsInfo * startNodeJacobian;
        const Eigen::Matrix3d trJsInfoJe = trJsInfo * endNodeJacobian;
        const Eigen::Matrix3d trJeInfoJe = trJeInfo * endNodeJacobian;

        /* Update the non-zero elements of sparse matrix H */
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                /* Compute the row index and column index */
                const int upRow = 3 * startNodeIdx + i;
                const int bottomRow = 3 * endNodeIdx + i;
                const int leftCol = 3 * startNodeIdx + j;
                const int rightCol = 3 * endNodeIdx + j;

                /* Append the element of J_i^T \Lambda_{ij} J_i */
                matATriplets.emplace_back(
                    upRow, leftCol, trJsInfoJs(i, j));
                /* Append the element of J_j^T \Lambda_{ij} J_j */
                matATriplets.emplace_back(
                    bottomRow, rightCol, trJeInfoJe(i, j));
                /* Append the element of J_i^T \Lambda_{ij} J_j */
                matATriplets.emplace_back(
                    upRow, rightCol, trJsInfoJe(i, j));
                /* Append the element of J_j^T \Lambda_{ij} J_i */
                matATriplets.emplace_back(
                    bottomRow, leftCol, trJsInfoJe(j, i));
            }
        }

        /* Update the elements of vector J^T \Lambda e */
        vecB.segment<3>(3 * startNodeIdx) += trJsInfo * errorVec;
        vecB.segment<3>(3 * endNodeIdx) += trJeInfo * errorVec;
    }

    /* Add the sufficiently large value to the diagonals of the
     * first 3x3 block of H to fix the increments of the
     * first node pose to zero */
    for (int i = 0; i < 3; ++i)
        matATriplets.emplace_back(i, i, 1e9);

    /* Add the damping factor (lambda) to the diagonals of H */
    for (int i = 0; i < numOfVariables; ++i)
        matATriplets.emplace_back(i, i, this->mLambda);

    /* Create a sparse matrix H */
    matA.setFromTriplets(matATriplets.cbegin(), matATriplets.cend());

    /* Solve the linear system */
    switch (this->mSolverType) {
        case SolverType::SparseCholesky: {
            /* Solve the linear system using sparse Cholesky factorization
             * and obtain the node pose increments */
            Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> spCholSolver;
            /* Compute a sparse Cholesky decomposition of matrix H */
            spCholSolver.compute(matA);
            /* Solve the linear system for increment */
            vecDelta = spCholSolver.solve(-vecB);
            break;
        }

        case SolverType::ConjugateGradient: {
            /* Solve the linear system using conjugate gradient method
             * and obtain the node pose increments */
            Eigen::ConjugateGradient<Eigen::SparseMatrix<double>> cgSolver;
            /* Initialize a conjugate gradient linear solver with matrix A */
            cgSolver.compute(matA);
            /* Solve the linear system for increment */
            vecDelta = cgSolver.solve(-vecB);
            break;
        }

        default: {
            /* Unknown solver type and program is aborted */
            assert(false && "Unknown linear solver type for optimization");
            break;
        }
    }

    /* Update the local map poses stored in pose graph */
    for (auto nodeIt = localMapNodes.begin();
         nodeIt != localMapNodes.end(); ++nodeIt) {
        const int nodeIdx = std::distance(localMapNodes.begin(), nodeIt);
        const RobotPose2D<double>& prevNodePose = nodeIt->mData.mGlobalPose;
        const RobotPose2D<double> newNodePose {
            prevNodePose.mX + vecDelta(3 * nodeIdx),
            prevNodePose.mY + vecDelta(3 * nodeIdx + 1),
            prevNodePose.mTheta + vecDelta(3 * nodeIdx + 2) };
        nodeIt->mData.mGlobalPose = newNodePose;
    }

    /* Update the scan node poses stored in the pose graph */
    for (auto nodeIt = scanNodes.begin();
         nodeIt != scanNodes.end(); ++nodeIt) {
        const int offsetToNode = std::distance(scanNodes.begin(), nodeIt);
        const int nodeIdx = numOfLocalMapNodes + offsetToNode;
        const RobotPose2D<double>& prevNodePose = nodeIt->mData.mGlobalPose;
        const RobotPose2D<double> newNodePose {
            prevNodePose.mX + vecDelta(3 * nodeIdx),
            prevNodePose.mY + vecDelta(3 * nodeIdx + 1),
            prevNodePose.mTheta + vecDelta(3 * nodeIdx + 2) };
        nodeIt->mData.mGlobalPose = newNodePose;
    }

    return;
}

/* Compute Jacobian matrices of the error function with respect to the
 * starting pose and ending pose of the pose graph edge */
void PoseGraphOptimizerLM::ComputeErrorJacobians(
    const RobotPose2D<double>& startNodePose,
    const RobotPose2D<double>& endNodePose,
    Eigen::Matrix3d& startNodeErrorJacobian,
    Eigen::Matrix3d& endNodeErrorJacobian) const
{
    /* Error function e_{ij} is defined as
     * e_{ij} = h(c_i, c_j) - \bar{z}_{ij} where
     * \bar{z}_{ij} is a measured offset between node c_i and c_j,
     * c_i = [x_i, y_i, \theta_i]^T and c_j = [x_j, y_j, \theta_j]^T are
     * the poses of the nodes to be optimized, and h(c_i, c_j) is a
     * function that computes the relative pose between c_i and c_j in
     * the frame of c_i */
    
    /* Function h(c_i, c_j) is written as follows:
     * h(c_i, c_j) = [ R_i^T (t_j - t_i),
     *                 \theta_j - \theta_i ]
     *             = [  cos \theta_i (x_j - x_i) + \sin \theta_i (y_j - y_i)
     *                 -sin \theta_i (x_j - x_i) + \cos \theta_i (y_j - y_i)
     *                  \theta_j - \theta_i ] */
    
    const double diffX = endNodePose.mX - startNodePose.mX;
    const double diffY = endNodePose.mY - startNodePose.mY;
    const double sinTheta = std::sin(startNodePose.mTheta);
    const double cosTheta = std::cos(startNodePose.mTheta);

    const double errorThetaPartialX = -sinTheta * diffX + cosTheta * diffY;
    const double errorThetaPartialY = -cosTheta * diffX - sinTheta * diffY;

    /* Compute a Jacobian matrix of the error function with respect to the
     * pose in the starting node */
    /* J_i = \frac{\partial e_{ij}}{\partial c_i}
     *     = [ -R_i^T, \frac{\partial R_i^T}{\partial \theta_i} (t_j - t_i)
     *          0,     -1 ]
     *     = [ -cos \theta_i, -sin \theta_i,  a
     *          sin \theta_i, -cos \theta_i,  b
     *          0,             0,            -1 ]
     * a = -sin \theta_i (x_j - x_i) + \cos \theta_i (y_j - y_i)
     * b = -cos \theta_i (x_j - x_i) - \sin \theta_i (y_j - y_i) */
    startNodeErrorJacobian << -cosTheta, -sinTheta, errorThetaPartialX,
                               sinTheta, -cosTheta, errorThetaPartialY,
                                    0.0,       0.0,               -1.0;
    
    /* Compute a Jacobian matrix of the error function with respect to the
     * pose in the end node */
    /* J_j = \frac{\partial e_{ij}}{\partial c_j}
     *     = [ R_i^T, 0
     *         0,     1 ]
     *     = [  cos \theta_i, sin \theta_i, 0
     *         -sin \theta_i, cos \theta_i, 0
     *          0,            0,            1 ] */
    endNodeErrorJacobian <<  cosTheta, sinTheta, 0.0,
                            -sinTheta, cosTheta, 0.0,
                                  0.0,      0.0, 1.0;
    
    return;
}

/* Compute error function */
void PoseGraphOptimizerLM::ComputeErrorFunction(
    const RobotPose2D<double>& startNodePose,
    const RobotPose2D<double>& endNodePose,
    const RobotPose2D<double>& edgeRelPose,
    Eigen::Vector3d& errorVec) const
{
    /* Compute the relative pose h(c_i, c_j) */
    const RobotPose2D<double> nodeRelPose =
        InverseCompound(startNodePose, endNodePose);
    
    /* Compute an error function e_{ij} = h(c_i, c_j) - \bar{z}_{ij} */
    errorVec << nodeRelPose.mX - edgeRelPose.mX,
                nodeRelPose.mY - edgeRelPose.mY,
                NormalizeAngle(nodeRelPose.mTheta - edgeRelPose.mTheta);

    return;
}

/* Compute total error */
double PoseGraphOptimizerLM::ComputeTotalError(
    const IdMap<LocalMapId, LocalMapNode>& localMapNodes,
    const IdMap<NodeId, ScanNode>& scanNodes,
    const std::vector<PoseGraphEdge>& poseGraphEdges) const
{
    double totalError = 0.0;

    /* Compute error function for each edge */
    for (const auto& edge : poseGraphEdges) {
        /* Retrieve the relative pose \bar{z}_{ij} from the edge */
        const RobotPose2D<double>& edgeRelPose = edge.mRelativePose;
        /* Retrieve the information matrix \Lambda_{ij} of the edge */
        const Eigen::Matrix3d& infoMat = edge.mInformationMat;

        /* Retrieve the poses of the start node and the end node */
        const LocalMapId startNodeId = edge.mLocalMapNodeId;
        const NodeId endNodeId = edge.mScanNodeId;

        const RobotPose2D<double>& startNodePose =
            localMapNodes.at(startNodeId).mGlobalPose;
        const RobotPose2D<double>& endNodePose =
            scanNodes.at(endNodeId).mGlobalPose;

        /* Compute the residual */
        Eigen::Vector3d errorVec;
        this->ComputeErrorFunction(startNodePose, endNodePose,
                                   edgeRelPose, errorVec);
        /* Compute the error value */
        const double errorVal = errorVec.transpose() * infoMat * errorVec;
        /* Apply the robust loss function */
        const double correctedError = this->mLossFunction->Loss(errorVal);

        /* Compute the error value */
        totalError += correctedError;
    }

    return totalError;
}

/* Dump the pose graph error */
void PoseGraphOptimizerLM::DumpError(
    const IdMap<LocalMapId, LocalMapNode>& localMapNodes,
    const IdMap<NodeId, ScanNode>& scanNodes,
    const std::vector<PoseGraphEdge>& poseGraphEdges) const
{
    auto* const pMetric = Metric::MetricManager::Instance();
    auto* const pErrorHistogram = pMetric->HistogramMetrics()("PoseGraphError");

    /* Reset the histogram for storing pose graph edge residuals */
    pErrorHistogram->Reset();

    /* Compute error function for each edge */
    for (const auto& edge : poseGraphEdges) {
        /* Retrieve the relative pose \bar{z}_{ij} from the edge */
        const RobotPose2D<double>& edgeRelPose = edge.mRelativePose;
        /* Retrieve the information matrix \Lambda_{ij} of the edge */
        const Eigen::Matrix3d& infoMat = edge.mInformationMat;

        /* Retrieve the poses of the start node and the end node */
        const LocalMapId startNodeId = edge.mLocalMapNodeId;
        const NodeId endNodeId = edge.mScanNodeId;

        const RobotPose2D<double>& startNodePose =
            localMapNodes.at(startNodeId).mGlobalPose;
        const RobotPose2D<double>& endNodePose =
            scanNodes.at(endNodeId).mGlobalPose;

        /* Compute the residual */
        Eigen::Vector3d errorVec;
        this->ComputeErrorFunction(startNodePose, endNodePose,
                                   edgeRelPose, errorVec);
        /* Compute the error value */
        const double errorVal = errorVec.transpose() * infoMat * errorVec;
        /* Add the error value to the error histogram */
        pErrorHistogram->Observe(errorVal);
    }

    /* Dump the error histogram */
    pErrorHistogram->Dump(std::cerr, true);

    return;
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
