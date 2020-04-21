
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
    CountingGridCell() : GridCell<T, bool>(),
                         mValue(Unknown),
                         mHit(0),
                         mMiss(0) { }
    /* Destructor */
    ~CountingGridCell() = default;

    /* Copy constructor */
    CountingGridCell(const CountingGridCell& other);
    /* Copy assignment operator */
    CountingGridCell& operator=(const CountingGridCell& other);
    /* Move constructor */
    CountingGridCell(CountingGridCell&& other) noexcept;
    /* Move assignment operator */
    CountingGridCell& operator=(CountingGridCell&& other) noexcept;

    /* Cast operator */
    explicit operator T() const override { return this->mValue; }
    /* Get the value of the grid cell */
    T Value() const override { return this->mValue; }

    /* Reset the grid cell state */
    void Reset() override;

    /* Update the value of the grid cell */
    void Update(const bool& hitOrMiss) override;

    /* Unknown occupancy probability value */
    using GridCell<T, bool>::Unknown;

    /* Smallest possible occupancy probability value,
     * which is slightly larger than the unknown value */
    static constexpr T Epsilon = static_cast<T>(1e-3);

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

/* Reset the grid cell state */
template <typename T>
void CountingGridCell<T>::Reset()
{
    this->mValue = Unknown;
    this->mHit = 0;
    this->mMiss = 0;
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
    
    /* If the number of the hits is zero, then add the small value
     * to distinguish from the unknown state */
    this->mValue = !this->mHit ? Epsilon : this->mValue;
}

} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_GRID_MAP_COUNTING_GRID_CELL_HPP */
