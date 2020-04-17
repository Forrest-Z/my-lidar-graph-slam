
/* motion_model_velocity.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_MOTION_MODEL_VELOCITY_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_MOTION_MODEL_VELOCITY_HPP

#include <cmath>
#include <random>

#include <Eigen/Core>

#include "my_lidar_graph_slam/pose.hpp"
#include "my_lidar_graph_slam/util.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/*
 * MotionModelVelocity class represents the simple velocity motion model
 */
class MotionModelVelocity
{
public:
    /*
     * ParameterType enum specifies the motion model parameter type
     */
    enum class ParameterType
    {
        /* Alpha coefficients are used as motion model parameters */
        AlphaCoefficient,
        /* Standard deviations are used as motion model parameters */
        StandardDeviation
    };

    /*
     * Parameters struct holds motion model parameters
     */
    struct Parameters
    {
        /* Type of the motion model parameter */
        ParameterType mType;

        union {
            struct {
                /* Coefficient of the squared translational velocity for
                 * calculating a translational velocity covariance */
                double               mAlphaTrans;
                /* Coefficient of the squared angular velocity for
                 * calculating a translational velocity covariance */
                double               mAlphaAngularToTrans;
                /* Coefficient of the squared translational velocity for
                 * calculating an angular velocity covariance */
                double               mAlphaTransToAngular;
                /* Coefficient of the squared angular velocity for
                 * calculating an angular velocity covariance */
                double               mAlphaAngular;
            } mAlphaCoeffs;

            struct {
                /* Standard deviation for a position (per 1 m translation) */
                double               mStdDevTrans;
                /* Standard deviation for a position (per 1 rad rotation) */
                double               mStdDevRotToTrans;
                /* Standard deviation for a angle (per 1 m translation) */
                double               mStdDevTransToRot;
                /* Standard deviation for a angle (per 1 rad rotation) */
                double               mStdDevRot;
            } mStdDevs;
        };
    };

public:
    /* Constructor */
    MotionModelVelocity(const Parameters& modelParams) :
        mParams(modelParams),
        mRandEngine(std::random_device()()) { }
    
    /* Destructor */
    ~MotionModelVelocity() = default;

    /* Copy constructor (disabled) */
    MotionModelVelocity(const MotionModelVelocity&) = delete;
    /* Copy assignment operator (disabled) */
    MotionModelVelocity& operator=(const MotionModelVelocity&) = delete;
    /* Move constructor (disabled) */
    MotionModelVelocity(MotionModelVelocity&&) = delete;
    /* Move assignment operator (disabled) */
    MotionModelVelocity& operator=(MotionModelVelocity&&) = delete;

    /* Sample a new pose */
    RobotPose2D<double> SamplePose(
        const RobotPose2D<double>& prevPose,
        const RobotPose2D<double>& relPose,
        const double timeDiff) const;
    
    /* Sample a new pose */
    RobotPose2D<double> SamplePose(
        const RobotPose2D<double>& prevPose,
        const double transVelocity,
        const double angularVelocity,
        const double timeDiff) const;
    
    /* Calculate a covariance matrix */
    Eigen::Matrix3d ComputeCovariance(
        const RobotPose2D<double>& prevPose,
        const RobotPose2D<double>& relPose,
        const double timeDiff,
        const Eigen::Matrix3d& prevPoseCovMat) const;

private:
    /* Sample a value from standard normal distribution */
    double StandardNormalDist() const;

    /* Calculate a translational velocity variance and
     * an angular velocity variance */
    void ComputeVelocityVariances(
        const double transVelocity,
        const double angularVelocity,
        const double timeDiff,
        double& transVelocityVar,
        double& angularVelocityVar) const;
    
    /* Calculate a velocity covariance matrix */
    Eigen::Matrix2d ComputeVelocityCovariance(
        const double transVelocity,
        const double angularVelocity,
        const double timeDiff) const;
    
    /* Calculate a Jacobian matrix of the new robot pose
     * with respect to the previous robot pose */
    Eigen::Matrix3d ComputePoseJacobian(
        const double prevPoseTheta,
        const double transVelocity,
        const double timeDiff) const;

    /* Calculate a Jacobian matrix of the new robot pose
     * with respect to the velocity */
    Eigen::Matrix<double, 3, 2> ComputeVelocityJacobian(
        const double prevPoseTheta,
        const double timeDiff) const;

private:
    /* Motion model parameters */
    Parameters           mParams;
    /* Pseudo-random number generator */
    mutable std::mt19937 mRandEngine;

    /* Minimum translational velocity */
    static constexpr double TransVelocityMin = 0.01;
    /* Minimum angular velocity */
    static constexpr double AngularVelocityMin = 0.01;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_MOTION_MODEL_VELOCITY_HPP */
