
/* motion_model_velocity.cpp */

#include "my_lidar_graph_slam/mapping/motion_model_velocity.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/* Sample a value from standard normal distribution */
double MotionModelVelocity::StandardNormalDist() const
{
    /* Standard normal distribution used for motion sampling */
    static std::normal_distribution stdNormalDist { 0.0, 1.0 };
    return stdNormalDist(this->mRandEngine);
}

/* Sample a new pose */
RobotPose2D<double> MotionModelVelocity::SamplePose(
    const RobotPose2D<double>& prevPose,
    const RobotPose2D<double>& relPose,
    const double timeDiff) const
{
    const double transDist = std::sqrt(
        relPose.mX * relPose.mX + relPose.mY * relPose.mY);
    const double transVelocity = std::max(
        MotionModelVelocity::TransVelocityMin, transDist / timeDiff);
    const double angularVelocity = std::max(
        MotionModelVelocity::AngularVelocityMin, relPose.mTheta / timeDiff);

    /* Sample a new pose using a translational velocity and
     * an angular velocity */
    return this->SamplePose(
        prevPose, transVelocity, angularVelocity, timeDiff);
}

/* Sample a new pose */
RobotPose2D<double> MotionModelVelocity::SamplePose(
    const RobotPose2D<double>& prevPose,
    const double transVelocity,
    const double angularVelocity,
    const double timeDiff) const
{
    /* Compute variances of a translational velocity and
     * an angular velocity */
    double transVelocityVar;
    double angularVelocityVar;
    this->ComputeVelocityVariances(transVelocity, angularVelocity, timeDiff,
                                   transVelocityVar, angularVelocityVar);

    /* Compute a translational velocity with a Gaussian noise */
    const double noisedTrans = transVelocity +
        std::sqrt(transVelocityVar) * this->StandardNormalDist();
    /* Compute an angular velocity with a Gaussian noise */
    const double noisedAngular = angularVelocity +
        std::sqrt(angularVelocityVar) * this->StandardNormalDist();
    
    /* If an angular velocity is almost zero, robot just goes straight */
    /* radius is infinity and thus cannot be calculated */
    if (std::fabs(noisedAngular) < 1e-4) {
        const double sampledTheta =
            NormalizeAngle(prevPose.mTheta + noisedAngular * timeDiff);
        const double sampledX =
            prevPose.mX + noisedTrans * std::cos(prevPose.mTheta) * timeDiff;
        const double sampledY =
            prevPose.mY + noisedTrans * std::sin(prevPose.mTheta) * timeDiff;
        
        return RobotPose2D<double> { sampledX, sampledY, sampledTheta };
    }

    /* Compute a new pose with noises */
    const double radius = noisedTrans / noisedAngular;
    const double sampledTheta =
        NormalizeAngle(prevPose.mTheta + noisedAngular * timeDiff);
    const double sampledX =
        prevPose.mX - radius * std::sin(prevPose.mTheta) +
        radius * std::sin(sampledTheta);
    const double sampledY =
        prevPose.mY + radius * std::cos(prevPose.mTheta) -
        radius * std::cos(sampledTheta);
    
    return RobotPose2D<double> { sampledX, sampledY, sampledTheta };
}

/* Calculate a motion covariance */
Eigen::Matrix3d MotionModelVelocity::ComputeCovariance(
    const RobotPose2D<double>& prevPose,
    const RobotPose2D<double>& relPose,
    const double timeDiff,
    const Eigen::Matrix3d& prevPoseCovMat) const
{
    /* A new pose \bm{x}_{t + 1} is calculated using
     * the previous pose \bm{x}_t and velocity \bm{u}_t as follows:
     * x_{t + 1} = \cos \theta_t (v_t \Delta_t) + x_t
     * y_{t + 1} = \sin \theta_t (v_t \Delta_t) + y_t
     * \theta_{t + 1} = (\omega_t \Delta_t) + \theta_t */

    /* \bm{x}_{t + 1}, \bm{x}_t, and \bm{u}_t are denoted as follows:
     * \bm{x}_{t + 1} = [x_{t + 1}, y_{t + 1}, \theta_{t + 1}]^T
     * \bm{x}_t = [x_t, y_t, \theta_t]^T
     * \bm{u}_t = [v_t \Delta_t, 0, \omega_t \Delta_t]^T */

    /* A covariance matrix \Sigma_{t + 1} of the new pose \bm{x}_{t + 1} is
     * calculated using a covariance matrix \Sigma_t of the previous pose
     * \bm{x}_t, a covariance matrix \Sigma_u of the velocity \bm{u}_t,
     * a Jacobian matrix J_{x_t} of the new pose with respect to
     * the previous pose \bm{x}_t, and a Jacobian matrix J_{u_t} of the
     * new pose with respect to the velocity \bm{u}_t as follows:
     * \Sigma_{t + 1} = S_0 + S_1
     * S_0 = J_{x_t} \Sigma_t J_{x_t}^T + J_{u_t}
     * S_1 = J_{u_t} \Sigma_u J_{u_t}^T */

    /* Calculate a translational velocity and an angular velocity */
    const double transDist = std::sqrt(
        relPose.mX * relPose.mX + relPose.mY * relPose.mY);
    const double transVelocity = std::max(
        MotionModelVelocity::TransVelocityMin, transDist / timeDiff);
    const double angularVelocity = std::max(
        MotionModelVelocity::AngularVelocityMin, relPose.mTheta / timeDiff);
    
    /* Compute a Jacobian matrix J_{x_t}, S_0 */
    const Eigen::Matrix3d poseJacobianMat =
        this->ComputePoseJacobian(prevPose.mTheta, transVelocity, timeDiff);
    const Eigen::Matrix3d poseComponent =
        poseJacobianMat * prevPoseCovMat * poseJacobianMat.transpose();
    
    /* Compute a covariance matrix \Sigma_u */
    const Eigen::Matrix2d velocityCovMat = this->ComputeVelocityCovariance(
        transVelocity, angularVelocity, timeDiff);
    
    /* Compute a Jacobian matrix J_{u_t}, S_1 */
    const Eigen::Matrix<double, 3, 2> velocityJacobianMat =
        this->ComputeVelocityJacobian(prevPose.mTheta, timeDiff);
    const Eigen::Matrix3d velocityComponent =
        velocityJacobianMat * velocityCovMat * velocityJacobianMat.transpose();
    
    /* Compute a covariance matrix */
    const Eigen::Matrix3d totalCovMat = poseComponent + velocityComponent;

    return totalCovMat;
}

/* Calculate a translational velocity variance and
 * an angular velocity variance */
void MotionModelVelocity::ComputeVelocityVariances(
    const double transVelocity,
    const double angularVelocity,
    const double timeDiff,
    double& transVelocityVar,
    double& angularVelocityVar) const
{
    if (this->mParams.mType == ParameterType::AlphaCoefficient) {
        /* Alpha coefficients are used as motion model parameters */
        const double transSq = transVelocity * transVelocity;
        const double angularSq = angularVelocity * angularVelocity;
        transVelocityVar =
            this->mParams.mAlphaCoeffs.mAlphaTrans * transSq +
            this->mParams.mAlphaCoeffs.mAlphaAngularToTrans * angularSq;
        angularVelocityVar =
            this->mParams.mAlphaCoeffs.mAlphaTransToAngular * transSq +
            this->mParams.mAlphaCoeffs.mAlphaAngular * angularSq;
        return;
    } else if (this->mParams.mType == ParameterType::StandardDeviation) {
        /* Standard deviations are used as motion model parameters */
        const double transSq =
            std::pow(this->mParams.mStdDevs.mStdDevTrans, 2.0);
        const double rotToTransSq =
            std::pow(this->mParams.mStdDevs.mStdDevRotToTrans, 2.0);
        const double transToRotSq =
            std::pow(this->mParams.mStdDevs.mStdDevTransToRot, 2.0);
        const double rotSq =
            std::pow(this->mParams.mStdDevs.mStdDevRot, 2.0);
        transVelocityVar =
            (transSq * std::abs(transVelocity) +
             rotToTransSq * std::abs(angularVelocity)) / timeDiff;
        angularVelocityVar =
            (transToRotSq * std::abs(transVelocity) +
             rotSq * std::abs(angularVelocity)) / timeDiff;
        return;
    }

    /* Otherwise return a small variance value */
    transVelocityVar = 1e-9;
    angularVelocityVar = 1e-9;
    
    return;
}

/* Calculate a velocity covariance matrix */
Eigen::Matrix2d MotionModelVelocity::ComputeVelocityCovariance(
    const double transVelocity,
    const double angularVelocity,
    const double timeDiff) const
{
    /* Compute a variance for a translational velocity
     * and an angular velocity */
    double transVelocityVar;
    double angularVelocityVar;
    this->ComputeVelocityVariances(transVelocity, angularVelocity, timeDiff,
                                   transVelocityVar, angularVelocityVar);

    /* Create a velocity covariance matrix */
    Eigen::Matrix2d velocityCovMat;
    velocityCovMat << transVelocityVar, 0.0,
                      0.0,              angularVelocityVar;
    
    return velocityCovMat;
}

/* Calculate a Jacobian matrix of the new robot pose
 * with respect to the previous robot pose */
Eigen::Matrix3d MotionModelVelocity::ComputePoseJacobian(
    const double prevPoseTheta,
    const double transVelocity,
    const double timeDiff) const
{
    const double cosTheta = std::cos(prevPoseTheta);
    const double sinTheta = std::sin(prevPoseTheta);

    /* Create a Jacobian matrix */
    Eigen::Matrix3d poseJacobianMat;
    poseJacobianMat << 1.0, 0.0, -transVelocity * timeDiff * sinTheta,
                       0.0, 1.0,  transVelocity * timeDiff * cosTheta,
                       0.0, 0.0,  1.0;
    
    return poseJacobianMat;
}

/* Calculate a Jacobian matrix of the new robot pose
 * with respect to the velocity */
Eigen::Matrix<double, 3, 2> MotionModelVelocity::ComputeVelocityJacobian(
    const double prevPoseTheta,
    const double timeDiff) const
{
    const double cosTheta = std::cos(prevPoseTheta);
    const double sinTheta = std::sin(prevPoseTheta);

    /* Create a Jacobian matrix */
    Eigen::Matrix<double, 3, 2> velocityJacobianMat;
    velocityJacobianMat << timeDiff * cosTheta, 0.0,
                           timeDiff * sinTheta, 0.0,
                           0.0,                 timeDiff;
    
    return velocityJacobianMat;
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
