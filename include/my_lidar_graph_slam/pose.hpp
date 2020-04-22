
/* pose.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_POSE_HPP
#define MY_LIDAR_GRAPH_SLAM_POSE_HPP

#include <cmath>
#include <iostream>
#include <type_traits>

namespace MyLidarGraphSlam {

template <typename T>
struct RobotPose2D final
{
    /* Default constructor */
    RobotPose2D() = default;

    /* Constructor with initial coordinates */
    RobotPose2D(T x, T y, T theta) :
        mX(x), mY(y), mTheta(theta) { }
    
    /* Cast operator */
    template <typename U, std::enable_if_t<
        std::is_constructible_v<U, T>, std::nullptr_t> = nullptr>
    inline explicit operator RobotPose2D<U>() const
    {
        return RobotPose2D<U> { static_cast<U>(this->mX),
                                static_cast<U>(this->mY),
                                static_cast<U>(this->mTheta) };
    }
    
    /* X-coordinate of the robot */
    T mX;
    /* Y-coordinate of the robot */
    T mY;
    /* Rotation angle (yaw angle) of the robot */
    T mTheta;

    /* Type of the coordinate */
    using ValueType = T;
};

/* Unary plus operator */
template <typename T>
RobotPose2D<T> operator+(const RobotPose2D<T>& lhs)
{
    return RobotPose2D { lhs.mX, lhs.mY, lhs.mTheta };
}

/* Unary minus operator */
template <typename T>
RobotPose2D<T> operator-(const RobotPose2D<T>& lhs)
{
    return RobotPose2D { -lhs.mX, -lhs.mY, -lhs.mTheta };
}

/* Plus operator */
template <typename T>
RobotPose2D<T> operator+(const RobotPose2D<T>& lhs, const RobotPose2D<T>& rhs)
{
    return RobotPose2D { lhs.mX + rhs.mX,
                         lhs.mY + rhs.mY,
                         lhs.mTheta + rhs.mTheta };
}

/* Minus operator */
template <typename T>
RobotPose2D<T> operator-(const RobotPose2D<T>& lhs, const RobotPose2D<T>& rhs)
{
    return RobotPose2D { lhs.mX - rhs.mX,
                         lhs.mY - rhs.mY,
                         lhs.mTheta - rhs.mTheta };
}

/* Multiply operator */
template <typename T>
RobotPose2D<T> operator*(const RobotPose2D<T>& lhs, const T& rhs)
{
    return RobotPose2D { lhs.mX * rhs,
                         lhs.mY * rhs,
                         lhs.mTheta * rhs };
}

/* Multiply operator */
template <typename T>
RobotPose2D<T> operator*(const T& lhs, const RobotPose2D<T>& rhs)
{
    return RobotPose2D { lhs * rhs.mX,
                         lhs * rhs.mY,
                         lhs * rhs.mTheta };
}

/* Divide operator */
template <typename T>
RobotPose2D<T> operator/(const RobotPose2D<T>& lhs, const T& rhs)
{
    return RobotPose2D { lhs.mX / rhs,
                         lhs.mY / rhs,
                         lhs.mTheta / rhs };
}

/* Equal operator */
template <typename T>
bool operator==(const RobotPose2D<T>& lhs, const RobotPose2D<T>& rhs)
{
    return (lhs.mX == rhs.mX) &&
           (lhs.mY == rhs.mY) &&
           (lhs.mTheta == rhs.mTheta);
}

/* Not equal operator */
template <typename T>
bool operator!=(const RobotPose2D<T>& lhs, const RobotPose2D<T>& rhs)
{
    return !operator==(lhs, rhs);
}

/* Calculate the distance between (0, 0) and the given pose */
template <typename T>
T Distance(const RobotPose2D<T>& p)
{
    return std::sqrt(p.mX * p.mX + p.mY * p.mY);
}

/* Calculate the distance between two given poses */
template <typename T>
T Distance(const RobotPose2D<T>& p0, const RobotPose2D<T>& p1)
{
    return std::sqrt((p0.mX - p1.mX) * (p0.mX - p1.mX) +
                     (p0.mY - p1.mY) * (p0.mY - p1.mY));
}

/* Calculate the squared distance between (0, 0) and the given pose */
template <typename T>
T SquaredDistance(const RobotPose2D<T>& p)
{
    return p.mX * p.mX + p.mY * p.mY;
}

/* Calculate the squared distance between two given poses */
template <typename T>
T SquaredDistance(const RobotPose2D<T>& p0, const RobotPose2D<T>& p1)
{
    return (p0.mX - p1.mX) * (p0.mX - p1.mX) +
           (p0.mY - p1.mY) * (p0.mY - p1.mY);
}

/* Compounding operator */
template <typename T>
RobotPose2D<T> Compound(const RobotPose2D<T>& startPose,
                        const RobotPose2D<T>& diffPose)
{
    T sinTheta = std::sin(startPose.mTheta);
    T cosTheta = std::cos(startPose.mTheta);

    T x = cosTheta * diffPose.mX - sinTheta * diffPose.mY + startPose.mX;
    T y = sinTheta * diffPose.mX + cosTheta * diffPose.mY + startPose.mY;
    T theta = startPose.mTheta + diffPose.mTheta;

    return RobotPose2D<T> { x, y, theta };
}

/* Inverse compounding operator */
template <typename T>
RobotPose2D<T> InverseCompound(const RobotPose2D<T>& startPose,
                               const RobotPose2D<T>& endPose)
{
    T sinTheta = std::sin(startPose.mTheta);
    T cosTheta = std::cos(startPose.mTheta);

    T dx = endPose.mX - startPose.mX;
    T dy = endPose.mY - startPose.mY;
    T dtheta = endPose.mTheta - startPose.mTheta;

    T x = cosTheta * dx + sinTheta * dy;
    T y = -sinTheta * dx + cosTheta * dy;
    T theta = dtheta;

    return RobotPose2D<T> { x, y, theta };
}

/* Move forward
 * Calculate Compound(startPose, diffPose) */
template <typename T>
RobotPose2D<T> MoveForward(const RobotPose2D<T>& startPose,
                           const RobotPose2D<T>& diffPose)
{
    /* Just use the compounding operator */
    return Compound(startPose, diffPose);
}

/* Move backward
 * Calculate the pose such that Compound(pose, diffPose) becomes endPose */
template <typename T>
RobotPose2D<T> MoveBackward(const RobotPose2D<T>& endPose,
                            const RobotPose2D<T>& diffPose)
{
    T theta = endPose.mTheta - diffPose.mTheta;
    T sinTheta = std::sin(theta);
    T cosTheta = std::cos(theta);

    T x = endPose.mX - cosTheta * diffPose.mX + sinTheta * diffPose.mY;
    T y = endPose.mY - sinTheta * diffPose.mX - cosTheta * diffPose.mY;

    return RobotPose2D<T> { x, y, theta };
}

/* Print to an output stream */
template <typename T>
std::ostream& operator<<(std::ostream& os, const RobotPose2D<T>& rhs)
{
    os << "[" << rhs.mX << ", " << rhs.mY << ", " << rhs.mTheta << "]";
    return os;
}

} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_POSE_HPP */
