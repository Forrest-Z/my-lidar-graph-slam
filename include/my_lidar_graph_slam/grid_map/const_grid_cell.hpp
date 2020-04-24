
/* const_grid_cell.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_GRID_MAP_CONST_GRID_CELL_HPP
#define MY_LIDAR_GRAPH_SLAM_GRID_MAP_CONST_GRID_CELL_HPP

#include "my_lidar_graph_slam/grid_map/grid_cell.hpp"

namespace MyLidarGraphSlam {

template <typename T>
class ConstGridCell final : public GridCell<T, double>
{
public:
    /* Default constructor */
    ConstGridCell() : GridCell<T, double>(),
                      mValue(Unknown) { }
    /* Destructor */
    ~ConstGridCell() = default;

    /* Default copy constructor */
    ConstGridCell(const ConstGridCell&) = default;
    /* Default copy assignment operator */
    ConstGridCell& operator=(const ConstGridCell&) = default;
    /* Default move constructor */
    ConstGridCell(ConstGridCell&&) noexcept = default;
    /* Default move assignment operator */
    ConstGridCell& operator=(ConstGridCell&&) noexcept = default;

    /* Cast operator */
    explicit operator T() const override { return this->mValue; }
    /* Get the occupancy probability value of the grid cell */
    T Value() const override { return this->mValue; }

    /* Reset the grid cell state */
    void Reset() override { this->mValue = Unknown; }

    /* Update the value of the grid cell */
    void Update(const double& newValue) override { this->mValue = newValue; }

    /* Unknown occupancy probability value */
    using GridCell<T, double>::Unknown;

private:
    /* Occupancy probability value */
    T mValue;
};

} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_GRID_MAP_CONST_GRID_CELL_HPP */
