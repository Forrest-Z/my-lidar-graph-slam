
/* util.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_UTIL_HPP
#define MY_LIDAR_GRAPH_SLAM_UTIL_HPP

#include <cmath>
#include <cstdint>
#include <type_traits>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "my_lidar_graph_slam/point.hpp"
#include "my_lidar_graph_slam/pose.hpp"

namespace MyLidarGraphSlam {

/* Convert strongly typed enum to integers */
template <typename T>
inline constexpr auto ToUnderlying(T enumValue) noexcept
{
    return static_cast<std::underlying_type_t<T>>(enumValue);
}

/* Convert rgb to unsigned 32-bit value */
inline std::uint32_t RGBToUInt32(
    std::uint8_t r, std::uint8_t g, std::uint8_t b)
{
    return static_cast<std::uint32_t>(
        (static_cast<std::uint32_t>(0)) |
        (static_cast<std::uint32_t>(r) << 16) |
        (static_cast<std::uint32_t>(g) << 8) |
        (static_cast<std::uint32_t>(b)));
}

/* Extract rgb components from unsigned 32-bit value */
inline std::uint8_t UInt32ToR(std::uint32_t rgb)
{
    return static_cast<std::uint8_t>((rgb & 0x00FF0000) >> 16);
}

/* Extract rgb components from unsigned 32-bit value */
inline std::uint8_t UInt32ToG(std::uint32_t rgb)
{
    return static_cast<std::uint8_t>((rgb & 0x0000FF00) >> 8);
}

/* Extract rgb components from unsigned 32-bit value */
inline std::uint8_t UInt32ToB(std::uint32_t rgb)
{
    return static_cast<std::uint8_t>((rgb & 0x000000FF));
}

/* Check if the number is power of 2 */
inline bool IsPowerOf2(int x)
{
    return (x != 0) && !(x & (x - 1));
}

/* Normalize the angle in radians from -pi to pi */
template <typename T, std::enable_if_t<
          std::is_arithmetic_v<T>, std::nullptr_t> = nullptr>
T NormalizeAngle(T theta)
{
    T normalizedTheta = std::fmod(theta, 2.0 * M_PI);

    if (normalizedTheta > M_PI)
        normalizedTheta -= 2.0 * M_PI;
    else if (normalizedTheta < -M_PI)
        normalizedTheta += 2.0 * M_PI;
    
    return normalizedTheta;
}

/* Normalize the angle in radians from -pi to pi */
template <typename T>
RobotPose2D<T> NormalizeAngle(const RobotPose2D<T>& robotPose)
{
    return RobotPose2D<T> { robotPose.mX,
                            robotPose.mY,
                            NormalizeAngle(robotPose.mTheta) };
}

/* Convert polar coordinate to cartesian coordinate */
template <typename T>
Point2D<T> ToCartesianCoordinate(T radius, T angle)
{
    return Point2D<T> { radius * std::cos(angle),
                        radius * std::sin(angle) };
}

/* Convert cartesian coordinate to polar coordinate */
template <typename T>
std::pair<double, double> ToPolarCoordinate(const Point2D<T>& point)
{
    double radius = std::sqrt(point.mX * point.mX + point.mY * point.mY);
    double angle = std::atan2(point.mY, point.mX);
    return std::make_pair(radius, angle);
}

/* Rotate a covariance matrix */
inline void RotateCovariance(
    const double rotationAngle,
    const Eigen::Matrix3d& covMat,
    Eigen::Matrix3d& rotatedCovMat)
{
    const double cosTheta = std::cos(rotationAngle);
    const double sinTheta = std::sin(rotationAngle);

    /* Create a rotation matrix */
    Eigen::Matrix3d rotationMat;
    rotationMat << cosTheta, -sinTheta, 0.0,
                   sinTheta,  cosTheta, 0.0,
                   0.0,       0.0,      1.0;
    
    /* Rotate a covariance matrix */
    rotatedCovMat = rotationMat * covMat * rotationMat.transpose();
}

/* Convert a covariance matrix from world frame to robot frame */
inline void ConvertCovarianceFromWorldToRobot(
    const RobotPose2D<double>& robotPose,
    const Eigen::Matrix3d& worldCovMat,
    Eigen::Matrix3d& robotCovMat)
{
    RotateCovariance(-robotPose.mTheta, worldCovMat, robotCovMat);
}

/* Convert a covariance matrix from robot frame to world frame */
inline void ConvertCovarianceFromRobotToWorld(
    const RobotPose2D<double>& robotPose,
    const Eigen::Matrix3d& robotCovMat,
    Eigen::Matrix3d& worldCovMat)
{
    RotateCovariance(robotPose.mTheta, robotCovMat, worldCovMat);
}

/* Execute bresenham algorithm */
template <typename T>
std::vector<Point2D<T>> Bresenham(
    const Point2D<T>& startIdx,
    const Point2D<T>& endIdx)
{
    std::vector<Point2D<T>> indices;

    int deltaX = endIdx.mX - startIdx.mX;
    int deltaY = endIdx.mY - startIdx.mY;
    int stepX = (deltaX < 0) ? -1 : 1;
    int stepY = (deltaY < 0) ? -1 : 1;
    int nextX = startIdx.mX;
    int nextY = startIdx.mY;

    deltaX = std::abs(deltaX * 2);
    deltaY = std::abs(deltaY * 2);

    /* Append the start cell index */
    indices.emplace_back(nextX, nextY);

    /* Execute Bresenham algorithm */
    if (deltaX > deltaY) {
        int err = deltaY - deltaX / 2;

        while (nextX != endIdx.mX) {
            if (err >= 0) {
                nextY += stepY;
                err -= deltaX;
            }
            nextX += stepX;
            err += deltaY;
            indices.emplace_back(nextX, nextY);
        }
    } else {
        int err = deltaX - deltaY / 2;

        while (nextY != endIdx.mY) {
            if (err >= 0) {
                nextX += stepX;
                err -= deltaY;
            }
            nextY += stepY;
            err += deltaX;
            indices.emplace_back(nextX, nextY);
        }
    }

    return indices;
}

} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_UTIL_HPP */
