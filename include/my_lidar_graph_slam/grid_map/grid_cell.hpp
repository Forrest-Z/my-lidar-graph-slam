
/* grid_cell.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_GRID_MAP_GRID_CELL_HPP
#define MY_LIDAR_GRAPH_SLAM_GRID_MAP_GRID_CELL_HPP

namespace MyLidarGraphSlam {

template <typename T, typename U>
class GridCell
{
public:
    /* Type definition */
    using ValueType = T;
    using ObservationType = U;
    
    /* Default constructor */
    GridCell() = default;
    /* Destructor */
    virtual ~GridCell() = default;

    /* Copy constructor */
    GridCell(const GridCell& other) = default;
    /* Move constructor */
    GridCell(GridCell&& other) = default;
    /* Copy assignment operator */
    GridCell& operator=(const GridCell& other) = default;
    /* Move assignment operator */
    GridCell& operator=(GridCell&& other) = default;

    /* Cast operator */
    virtual explicit operator T() const = 0;
    /* Get the value of the grid cell */
    virtual T Value() const = 0;

    /* Reset the grid cell state */
    virtual void Reset() = 0;
    
    /* Update the value of the grid cell */
    virtual void Update(const U& currentObservation) = 0;

    /* Unknown occupancy probability value (negative) */
    static constexpr T Unknown = static_cast<T>(-1.0);
};

} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_GRID_MAP_GRID_CELL_HPP */
