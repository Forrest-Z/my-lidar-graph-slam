
/* grid_map_base.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_GRID_MAP_GRID_MAP_BASE_HPP
#define MY_LIDAR_GRAPH_SLAM_GRID_MAP_GRID_MAP_BASE_HPP

#include "my_lidar_graph_slam/point.hpp"
#include "my_lidar_graph_slam/pose.hpp"

namespace MyLidarGraphSlam {

template <typename T>
class GridMapBase
{
public:
    /* Constructor */
    GridMapBase() = default;
    /* Destructor */
    virtual ~GridMapBase() = default;

    /* Copy constructor (disabled) */
    GridMapBase(const GridMapBase&) = delete;
    /* Copy assignment operator (disabled) */
    GridMapBase& operator=(const GridMapBase&) = delete;

    /* Move constructor */
    GridMapBase(GridMapBase&&) noexcept = default;
    /* Move assignment operator */
    GridMapBase& operator=(GridMapBase&&) noexcept = default;

    /* Get the unknown occupancy probability value */
    virtual T UnknownValue() const = 0;

    /* Check if the grid cell index is inside the map */
    virtual bool IsInside(int idxX, int idxY) const = 0;
    /* Check if the grid cell index is inside the map */
    virtual bool IsInside(const Point2D<int>& gridCellIdx) const = 0;

    /* Check if the point in world frame is inside the map */
    virtual bool IsInside(double mapX, double mapY) const = 0;
    /* Check if the point in world frame is inside the map */
    virtual bool IsInside(const Point2D<double>& mapPos) const = 0;

    /* Check if the grid cell is allocated on the heap */
    virtual bool IsAllocated(int idxX, int idxY) const = 0;
    /* Check if the grid cell is allocated on the heap */
    virtual bool IsAllocated(const Point2D<int>& gridCellIdx) const = 0;

    /* Convert the grid cell index into the point in world frame
     * The returned point is the bottom-left of the grid cell */
    virtual Point2D<double> GridCellIndexToWorldCoordinate(
        int idxX, int idxY) const = 0;
    /* Convert the grid cell index into the point in world frame
     * The returned point is the bottom-left of the grid cell */
    virtual Point2D<double> GridCellIndexToWorldCoordinate(
        const Point2D<int>& gridCellIdx) const = 0;

    /* Convert the point in world frame to the grid cell index */
    virtual Point2D<int> WorldCoordinateToGridCellIndex(
        double mapX, double mapY) const = 0;
    /* Convert the point in world frame to the grid cell index */
    virtual Point2D<int> WorldCoordinateToGridCellIndex(
        const Point2D<double>& mapPos) const = 0;

    /* Convert the point in world frame to the
     * floating-point grid cell index */
    virtual Point2D<double> WorldCoordinateToGridCellIndexFloat(
        double mapX, double mapY) const = 0;
    /* Convert the point in world frame to the
     * floating-point grid cell index */
    virtual Point2D<double> WorldCoordinateToGridCellIndexFloat(
        const Point2D<double>& mapPos) const = 0;
    
    /* Get the occupancy probability value of the specified grid cell */
    virtual T Value(int idxX, int idxY) const = 0;
    /* Get the occupancy probability value of the specified grid cell */
    virtual T Value(const Point2D<int>& gridCellIdx) const = 0;

    /* Get the occupancy probability value of the specified grid cell
     * The default value is returned if the specified grid cell is
     * out of bounds or is not yet allocated */
    virtual T Value(int idxX, int idxY, T defaultVal) const = 0;
    /* Get the occupancy probability value of the specified grid cell */
    virtual T Value(const Point2D<int>& gridCellIdx, T defaultVal) const = 0;

    /* Calculate the distance between two grid cells */
    virtual double Distance(int idxX0, int idxY0,
                            int idxX1, int idxY1) const = 0;
    /* Calculate the distance between two grid cells */
    virtual double Distance(const Point2D<int>& gridCellIdx0,
                            const Point2D<int>& gridCellIdx1) const = 0;
    
    /* Calculate the squared distance between two grid cells */
    virtual double SquaredDistance(int idxX0, int idxY0,
                                   int idxX1, int idxY1) const = 0;
    /* Calculate the squared distance between two grid cells */
    virtual double SquaredDistance(const Point2D<int>& gridCellIdx0,
                                   const Point2D<int>& gridCellIdx1) const = 0;

    /* Get the map resolution (grid cell size in meters) */
    virtual double Resolution() const = 0;

    /* Get the number of the grid cells (horizontal) */
    virtual int NumOfGridCellsX() const = 0;
    /* Get the number of the grid cells (vertical) */
    virtual int NumOfGridCellsY() const = 0;

    /* Get the size of the map in meters (horizontal) */
    virtual double MapSizeX() const = 0;
    /* Get the size of the map in meters (vertical) */
    virtual double MapSizeY() const = 0;

    /* Get the minimum position of the map in world coordinate */
    virtual const Point2D<double>& MinPos() const = 0;
};

} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_GRID_MAP_GRID_MAP_BASE_HPP */
