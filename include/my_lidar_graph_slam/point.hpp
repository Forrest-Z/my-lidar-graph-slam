
/* point.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_POINT_HPP
#define MY_LIDAR_GRAPH_SLAM_POINT_HPP

#include <cmath>
#include <iostream>
#include <type_traits>

namespace MyLidarGraphSlam {

template <typename T>
struct Point2D final
{
    /* Type of the coordinate */
    using ValueType = T;

    /* Default constructor */
    Point2D() = default;

    /* Constructor with initial coordinates */
    constexpr Point2D(T x, T y) :
        mX(x), mY(y) { }

    /* Cast operator */
    template <typename U, std::enable_if_t<
        std::is_constructible_v<U, T>, std::nullptr_t> = nullptr>
    inline explicit operator Point2D<U>() const
    {
        return Point2D<U> { static_cast<U>(this->mX),
                            static_cast<U>(this->mY) };
    }

    /* Origin point */
    static constexpr Point2D Zero { 0.0, 0.0 };

    /* X-coordinate of the point */
    T mX;
    /* Y-coordinate of the point */
    T mY;
};

/* Unary plus operator */
template <typename T>
Point2D<T> operator+(const Point2D<T>& lhs)
{
    return Point2D<T> { lhs.mX, lhs.mY };
}

/* Unary minus operator */
template <typename T>
Point2D<T> operator-(const Point2D<T>& lhs)
{
    return Point2D<T> { -lhs.mX, -lhs.mY };
}

/* Plus operator */
template <typename T>
Point2D<T> operator+(const Point2D<T>& lhs, const Point2D<T>& rhs)
{
    return Point2D<T> { lhs.mX + rhs.mX, lhs.mY + rhs.mY };
}

/* Minus operator */
template <typename T>
Point2D<T> operator-(const Point2D<T>& lhs, const Point2D<T>& rhs)
{
    return Point2D<T> { lhs.mX - rhs.mX, lhs.mY - rhs.mY };
}

/* Multiply operator */
template <typename T>
Point2D<T> operator*(const Point2D<T>& lhs, const T& rhs)
{
    return Point2D<T> { lhs.mX * rhs, lhs.mY * rhs };
}

/* Multiply operator */
template <typename T>
Point2D<T> operator*(const T& lhs, const Point2D<T>& rhs)
{
    return Point2D<T> { lhs * rhs.mX, lhs * rhs.mY };
}

/* Divide operator */
template <typename T>
Point2D<T> operator/(const Point2D<T>& lhs, const T& rhs)
{
    return Point2D<T> { lhs.mX / rhs, lhs.mY / rhs };
}

/* Equal operator */
template <typename T>
bool operator==(const Point2D<T>& lhs, const Point2D<T>& rhs)
{
    return (lhs.mX == rhs.mX) && (lhs.mY == rhs.mY);
}

/* Not equal operator */
template <typename T>
bool operator!=(const Point2D<T>& lhs, const Point2D<T>& rhs)
{
    return !operator==(lhs, rhs);
}

/* Calculate the distance between (0, 0) and the given point */
template <typename T>
T Distance(const Point2D<T>& p)
{
    return std::sqrt(p.mX * p.mX + p.mY * p.mY);
}

/* Calculate the distance between two given points */
template <typename T>
T Distance(const Point2D<T>& p0, const Point2D<T>& p1)
{
    return std::sqrt((p0.mX - p1.mX) * (p0.mX - p1.mX) +
                     (p0.mY - p1.mY) * (p0.mY - p1.mY));
}

/* Calculate the squared distance between (0, 0) and the given point */
template <typename T>
T SquaredDistance(const Point2D<T>& p)
{
    return p.mX * p.mX + p.mY * p.mY;
}

/* Calculate the squared distance between two given points */
template <typename T>
T SquaredDistance(const Point2D<T>& p0, const Point2D<T>& p1)
{
    return (p0.mX - p1.mX) * (p0.mX - p1.mX) +
           (p0.mY - p1.mY) * (p0.mY - p1.mY);
}

/* Print to an output stream */
template <typename T>
std::ostream& operator<<(std::ostream& os, const Point2D<int>& rhs)
{
    os << "[" << rhs.mX << ", " << rhs.mY << "]";
    return os;
}

} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_POINT_HPP */
