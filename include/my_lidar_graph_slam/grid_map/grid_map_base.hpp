
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

    /* Copy constructor */
    GridMapBase(const GridMapBase&) = default;
    /* Copy assignment operator */
    GridMapBase& operator=(const GridMapBase&) = default;
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

    /* Check if the point in the local frame is inside the map */
    virtual bool IsInside(double localMapX, double localMapY) const = 0;
    /* Check if the point in the local frame is inside the map */
    virtual bool IsInside(const Point2D<double>& localMapPos) const = 0;

    /* Check if the grid cell is allocated on the heap */
    virtual bool IsAllocated(int idxX, int idxY) const = 0;
    /* Check if the grid cell is allocated on the heap */
    virtual bool IsAllocated(const Point2D<int>& gridCellIdx) const = 0;

    /* Convert the grid cell index into the point in the local frame
     * The returned point is the bottom-left of the grid cell */
    virtual Point2D<double> GridCellIndexToLocalPos(
        int idxX, int idxY) const = 0;
    /* Convert the grid cell index into the point in the local frame
     * The returned point is the bottom-left of the grid cell */
    virtual Point2D<double> GridCellIndexToLocalPos(
        const Point2D<int>& gridCellIdx) const = 0;

    /* Convert the point in the local frame to the grid cell index */
    virtual Point2D<int> LocalPosToGridCellIndex(
        double localMapX, double localMapY) const = 0;
    /* Convert the point in the local frame to the grid cell index */
    virtual Point2D<int> LocalPosToGridCellIndex(
        const Point2D<double>& localMapPos) const = 0;

    /* Convert the point in the local frame to the
     * floating-point grid cell index */
    virtual Point2D<double> LocalPosToGridCellIndexFloat(
        double localMapX, double localMapY) const = 0;
    /* Convert the point in the local frame to the
     * floating-point grid cell index */
    virtual Point2D<double> LocalPosToGridCellIndexFloat(
        const Point2D<double>& localMapPos) const = 0;

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

    /* Get the minimum position of the map in the local coordinate */
    virtual const Point2D<double>& LocalMinPos() const = 0;
};

} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_GRID_MAP_GRID_MAP_BASE_HPP */
