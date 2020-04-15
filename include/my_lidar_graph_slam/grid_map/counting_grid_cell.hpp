
/* counting_grid_cell.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_GRID_MAP_COUNTING_GRID_CELL_HPP
#define MY_LIDAR_GRAPH_SLAM_GRID_MAP_COUNTING_GRID_CELL_HPP

#include "my_lidar_graph_slam/grid_map/grid_cell.hpp"

#include <cstdint>
#include <utility>

namespace MyLidarGraphSlam {

template <typename T>
class CountingGridCell final : public GridCell<T, bool>
{
public:
    /* Default constructor */
    CountingGridCell() :
        GridCell<T, bool>(),
        mValue(CountingGridCell::Unknown),
        mHit(0),
        mMiss(0) { }
    /* Destructor */
    ~CountingGridCell() = default;

    /* Copy constructor */
    CountingGridCell(const CountingGridCell& other);
    /* Move constructor */
    CountingGridCell(CountingGridCell&& other) noexcept;
    /* Copy assignment operator */
    CountingGridCell& operator=(const CountingGridCell& other);
    /* Move assignment operator */
    CountingGridCell& operator=(CountingGridCell&& other) noexcept;

    /* Cast operator */
    explicit operator T() const override { return this->mValue; }
    /* Get the value of the grid cell */
    T Value() const override { return this->mValue; }

    /* Update the value of the grid cell */
    void Update(const bool& hitOrMiss) override;

private:
    T             mValue;
    std::uint32_t mHit;
    std::uint32_t mMiss;
};

/* Copy constructor */
template <typename T>
CountingGridCell<T>::CountingGridCell(
    const CountingGridCell<T>& other) :
    GridCell<T, bool>(),
    mValue(other.mValue),
    mHit(other.mHit),
    mMiss(other.mMiss)
{
}

/* Move constructor */
template <typename T>
CountingGridCell<T>::CountingGridCell(
    CountingGridCell<T>&& other) noexcept :
    GridCell<T, bool>(),
    mValue(std::move(other.mValue)),
    mHit(std::move(other.mHit)),
    mMiss(std::move(other.mMiss))
{
}

/* Copy assignment operator */
template <typename T>
CountingGridCell<T>& CountingGridCell<T>::operator=(
    const CountingGridCell<T>& other)
{
    if (this == &other)
        return *this;
    
    this->mValue = other.mValue;
    this->mHit = other.mHit;
    this->mMiss = other.mMiss;

    return *this;
}

/* Move assignment operator */
template <typename T>
CountingGridCell<T>& CountingGridCell<T>::operator=(
    CountingGridCell<T>&& other) noexcept
{
    if (this == &other)
        return *this;
    
    this->mValue = std::move(other.mValue);
    this->mHit = std::move(other.mHit);
    this->mMiss = std::move(other.mMiss);

    return *this;
}

/* Update the value of the grid cell */
template <typename T>
void CountingGridCell<T>::Update(const bool& hitOrMiss)
{
    /* Update the counter */
    if (hitOrMiss)
        ++this->mHit;
    else
        ++this->mMiss;
    
    /* Update the occupancy probability value */
    this->mValue = static_cast<T>(this->mHit) /
                   static_cast<T>(this->mHit + this->mMiss);
}

} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_GRID_MAP_COUNTING_GRID_CELL_HPP */
