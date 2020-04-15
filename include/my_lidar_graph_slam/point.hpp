
/* point.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_POINT_HPP
#define MY_LIDAR_GRAPH_SLAM_POINT_HPP

#include <iostream>
#include <type_traits>

namespace MyLidarGraphSlam {

template <typename T>
struct Point2D final
{
    /* Default constructor */
    Point2D() = default;

    /* Constructor with initial coordinates */
    Point2D(T x, T y) :
        mX(x), mY(y) { }
    
    /* Cast operator */
    template <typename U, std::enable_if_t<
        std::is_constructible_v<U, T>, std::nullptr_t> = nullptr>
    inline explicit operator Point2D<U>() const
    {
        return Point2D<U> { static_cast<U>(this->mX),
                            static_cast<U>(this->mY) };
    }

    /* X-coordinate of the point */
    T mX;
    /* Y-coordinate of the point */
    T mY;

    /* Type of the coordinate */
    using ValueType = T;
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

/* Print to an output stream */
template <typename T>
std::ostream& operator<<(std::ostream& os, const Point2D<int>& rhs)
{
    os << "[" << rhs.mX << ", " << rhs.mY << "]";
    return os;
}

} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_POINT_HPP */
