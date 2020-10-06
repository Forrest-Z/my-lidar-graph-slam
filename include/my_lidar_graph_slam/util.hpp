
/* util.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_UTIL_HPP
#define MY_LIDAR_GRAPH_SLAM_UTIL_HPP

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <functional>
#include <iostream>
#include <iterator>
#include <sstream>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "my_lidar_graph_slam/point.hpp"
#include "my_lidar_graph_slam/pose.hpp"

/* Assert macro with a custom message (nullptr is allowed) */
#define XAssert(predicate, message) \
    (!(predicate) && AssertHandler(#predicate, __FILE__, __LINE__, message))
/* Assert macro with no custom message */
#define Assert(predicate) \
    XAssert(predicate, nullptr)

/* Assert function that is enabled in Release mode */
inline bool AssertHandler(const char* predicateExpression,
                          const char* fileName,
                          const int lineNumber,
                          const char* message)
{
    const char* customMessage = message != nullptr ? message : "";
    std::cerr << "Assertion failed: " << predicateExpression << '\n'
              << "File: " << fileName << ", "
              << "Line: " << lineNumber << ", "
              << "Message: " << customMessage << '\n';
    std::abort();
    return true;
}

namespace MyLidarGraphSlam {

/* Join elements in a container with a delimiter */
template <class C, class V = typename C::value_type>
std::string Join(const C& elements, const char* delimiter)
{
    std::ostringstream strStream;
    auto beginIt = std::begin(elements);
    auto endIt = std::end(elements);

    /* Print the elements if the container is not empty */
    if (beginIt != endIt) {
        std::copy(beginIt, std::prev(endIt),
                  std::ostream_iterator<V>(strStream, delimiter));
        beginIt = std::prev(endIt);
    }

    /* Print the last remaining element if the container is not empty */
    if (beginIt != endIt)
        strStream << *beginIt;

    return strStream.str();
}

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

/* Convert nanoseconds to milliseconds */
inline double ToMilliSeconds(std::int_least64_t nanoSec)
{
    const auto nanoSeconds = std::chrono::nanoseconds(nanoSec);
    const auto milliSeconds = std::chrono::duration_cast<
        std::chrono::milliseconds>(nanoSeconds);
    return milliSeconds.count();
}

/* Convert nanoseconds to microseconds */
inline double ToMicroSeconds(std::int_least64_t nanoSec)
{
    const auto nanoSeconds = std::chrono::nanoseconds(nanoSec);
    const auto microSeconds = std::chrono::duration_cast<
        std::chrono::microseconds>(nanoSeconds);
    return microSeconds.count();
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
inline Eigen::Matrix3d RotateCovariance(
    const double rotationAngle,
    const Eigen::Matrix3d& covMat)
{
    const double cosTheta = std::cos(rotationAngle);
    const double sinTheta = std::sin(rotationAngle);

    /* Create a rotation matrix */
    Eigen::Matrix3d rotationMat;
    rotationMat << cosTheta, -sinTheta, 0.0,
                   sinTheta,  cosTheta, 0.0,
                   0.0,       0.0,      1.0;
    
    /* Rotate a covariance matrix */
    return rotationMat * covMat * rotationMat.transpose();
}

/* Convert a covariance matrix from world frame to robot frame */
inline Eigen::Matrix3d ConvertCovarianceFromWorldToLocal(
    const RobotPose2D<double>& poseToLocalFrame,
    const Eigen::Matrix3d& worldCovMat)
{
    return RotateCovariance(-poseToLocalFrame.mTheta, worldCovMat);
}

/* Convert a covariance matrix from local frame to world frame */
inline Eigen::Matrix3d ConvertCovarianceFromLocalToWorld(
    const RobotPose2D<double>& poseToLocalFrame,
    const Eigen::Matrix3d& localCovMat)
{
    return RotateCovariance(poseToLocalFrame.mTheta, localCovMat);
}

/* Execute sliding window maximum problem */
template <typename T>
void SlidingWindowMax(std::function<T(int)> inFunc,
                      std::function<void(int, T)> outFunc,
                      int numOfElements,
                      int winSize)
{
    /* Source code was taken from https://www.geeksforgeeks.org/ and
     * slightly modified (zero padding added) */

    /* Create a double ended queue, idxQueue that will store indexes of
     * array elements
     * idxQueue will store indexes of useful elements in every window and
     * it will maintain decreasing order of values from front to rear in
     * idxQueue, i.e., inFunc(idxQueue.front()) to inFunc(idxQueue.back()) are
     * sorted in decreasing order */
    std::deque<int> idxQueue;

    int idxIn = 0;
    int idxOut = 0;

    /* Process the first winSize (or first window) elements of array */
    for (idxIn = 0; idxIn < winSize; ++idxIn) {
        /* For every element, the previous smaller elements are useless so
         * remove them from idxQueue */
        while ((!idxQueue.empty()) && inFunc(idxIn) >= inFunc(idxQueue.back()))
            idxQueue.pop_back();
        
        /* Add new element at rear of idxQueue */
        idxQueue.push_back(idxIn);
    }

    /* Now idxQueue.front() contains the index of the maximum element
     * in the first window */

    /* Process the rest of the elements */
    for (; idxIn < numOfElements; ++idxIn) {
        /* The element at the front of the queue is the maximum element of
         * the previous window */
        outFunc(idxOut++, inFunc(idxQueue.front()));

        /* Remove the elements which are out of the current window */
        while ((!idxQueue.empty()) && idxQueue.front() <= idxIn - winSize)
            idxQueue.pop_front();
        
        /* Remove all elements smaller than the current element */
        while ((!idxQueue.empty()) && inFunc(idxIn) >= inFunc(idxQueue.back()))
            idxQueue.pop_back();
        
        /* Add the current element at the rear of the queue */
        idxQueue.push_back(idxIn);
    }

    /* Repeat the last elements */
    for (; idxOut < numOfElements; ++idxOut)
        outFunc(idxOut, inFunc(idxQueue.front()));
}

/* Execute bresenham algorithm */
template <typename T>
std::vector<Point2D<T>> Bresenham(const Point2D<T>& startIdx,
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
