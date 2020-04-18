
/* grid_map.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_GRID_MAP_GRID_MAP_HPP
#define MY_LIDAR_GRAPH_SLAM_GRID_MAP_GRID_MAP_HPP

#include <algorithm>
#include <cassert>
#include <cmath>
#include <memory>

#include "my_lidar_graph_slam/point.hpp"
#include "my_lidar_graph_slam/pose.hpp"
#include "my_lidar_graph_slam/util.hpp"
#include "my_lidar_graph_slam/grid_map/grid_cell.hpp"
#include "my_lidar_graph_slam/grid_map/grid_map_patch.hpp"

namespace MyLidarGraphSlam {

template <typename T>
class GridMap final
{
public:
    /* Type definitions */
    using GridCellType = T;
    using PatchType = Patch<T>;
    using ValueType = typename T::ValueType;
    using ObservationType = typename T::ObservationType;

    /* Constructor with number of grid cells */
    GridMap(double mapResolution,
            int patchSize,
            int initialNumOfCellsX,
            int initialNumOfCellsY,
            const Point2D<double>& centerPos);

    /* Constructor with map size */
    GridMap(double mapResolution,
            int patchSize,
            double mapSizeX,
            double mapSizeY,
            const Point2D<double>& centerPos);
    
    /* Constructor with positions */
    GridMap(double mapResolution,
            int patchSize,
            double minX,
            double minY,
            double maxX,
            double maxY);
    
    /* Destructor */
    ~GridMap() = default;

    /* Copy constructor (disabled) */
    GridMap(const GridMap&) = delete;
    /* Copy assignment operator (disabled) */
    GridMap& operator=(const GridMap&) = delete;

    /* Move constructor */
    GridMap(GridMap&& other) noexcept;
    /* Move assignment operator */
    GridMap& operator=(GridMap&& other) noexcept;

    /* Resize the map */
    void Resize(double minX, double minY,
                double maxX, double maxY);
    /* Expand the map if necessary */
    void Expand(double minX, double minY,
                double maxX, double maxY,
                double enlargeStep = 5.0);
    
    /* Reset occupancy probability values of all grid cells */
    void Reset();
 
    /* Check if the grid cell index is inside the map */
    inline bool IsInside(int gridCellX, int gridCellY) const;
    /* Check if the grid cell index is inside the map */
    inline bool IsInside(const Point2D<int>& gridCellIdx) const
    { return this->IsInside(gridCellIdx.mX, gridCellIdx.mY); }

    /* Check if the point in world frame is inside the map */
    inline bool IsInside(double mapX, double mapY) const
    { return this->IsInside(
        this->WorldCoordinateToGridCellIndex(mapX, mapY)); }
    /* Check if the point in world frame is inside the map */
    inline bool IsInside(const Point2D<double>& mapPos) const
    { return this->IsInside(mapPos.mX, mapPos.mY); }

    /* Convert the grid cell index into the point in world frame
     * The returned point is the bottom-left of the grid cell */
    inline Point2D<double> GridCellIndexToWorldCoordinate(
        int gridCellIdxX, int gridCellIdxY) const;
    /* Convert the grid cell index into the point in world frame
     * The returned point is the bottom-left of the grid cell */
    inline Point2D<double> GridCellIndexToWorldCoordinate(
        const Point2D<int>& gridCellIdx) const
    { return this->GridCellIndexToWorldCoordinate(
        gridCellIdx.mX, gridCellIdx.mY); }
    
    /* Convert the point in world frame to the grid cell index */
    inline Point2D<int> WorldCoordinateToGridCellIndex(
        double mapX, double mapY) const;
    /* Convert the point in world frame to the grid cell index */
    inline Point2D<int> WorldCoordinateToGridCellIndex(
        const Point2D<double>& mapPos) const
    { return this->WorldCoordinateToGridCellIndex(mapPos.mX, mapPos.mY); }

    /* Get the grid cell of the specified index */
    inline GridCellType& GridCellAt(int gridCellX, int gridCellY);
    /* Get the grid cell of the specified index */
    inline GridCellType& GridCellAt(const Point2D<int>& gridCellIdx)
    { return this->GridCellAt(gridCellIdx.mX, gridCellIdx.mY); }

    /* Get the grid cell of the specified index */
    inline const GridCellType& GridCellAt(
        int gridCellX, int gridCellY) const;
    /* Get the grid cell of the specified index */
    inline const GridCellType& GridCellAt(
        const Point2D<int>& gridCellIdx) const
    { return this->GridCellAt(gridCellIdx.mX, gridCellIdx.mY); }

    /* Get the grid cell of the specified point */
    inline GridCellType& GridCellAt(double mapX, double mapY)
    { return this->GridCellAt(
        this->WorldCoordinateToGridCellIndex(mapX, mapY)); }
    /* Get the grid cell of the specified point */
    inline GridCellType& GridCellAt(const Point2D<double>& mapPos)
    { return this->GridCellAt(mapPos.mX, mapPos.mY); }

    /* Get the grid cell of the specified point */
    inline const GridCellType& GridCellAt(double mapX, double mapY) const
    { return this->GridCellAt(
        this->WorldCoordinateToGridCellIndex(mapX, mapY)); }
    /* Get the grid cell of the specified point */
    inline const GridCellType& GridCellAt(const Point2D<double>& mapPos) const
    { return this->GridCellAt(mapPos.mX, mapPos.mY); }

    /* Get the occupancy probability value of the specified grid cell */
    inline ValueType Value(int gridCellX, int gridCellY) const;
    /* Get the occupancy probability value of the specified grid cell */
    inline ValueType Value(const Point2D<int>& gridCellIdx) const
    { return this->Value(gridCellIdx.mX, gridCellIdx.mY); }

    /* Get the occupancy probability value of the specified grid cell
     * The default value is returned if the specified grid cell is
     * out of bounds or is not yet allocated */
    inline ValueType Value(int gridCellX, int gridCellY,
                           ValueType defaultValue) const;
    /* Get the occupancy probability value of the specified grid cell */
    inline ValueType Value(const Point2D<int>& gridCellIdx,
                           ValueType defaultValue) const
    { return this->Value(gridCellIdx.mX, gridCellIdx.mY, defaultValue); }

    /* Update the occupancy probability value of the specified grid cell */
    inline void Update(int gridCellX, int gridCellY,
                       const ObservationType& latestObservation);
    /* Update the occupancy probability value of the specified grid cell */
    inline void Update(const Point2D<int>& gridCellIdx,
                       const ObservationType& latestObservation)
    { this->Update(gridCellIdx.mX, gridCellIdx.mY, latestObservation); }

    /* Update the occupancy probability value of the specified point */
    inline void Update(double mapX, double mapY,
                       const ObservationType& latestObservation)
    { this->Update(this->WorldCoordinateToGridCellIndex(mapX, mapY),
                   latestObservation); }
    /* Update the occupancy probability value of the specified point */
    inline void Update(const Point2D<double>& mapPos,
                       const ObservationType& latestObservation)
    { this->Update(mapPos.mX, mapPos.mY, latestObservation); }

    /* Calculate the distance between two grid cells */
    inline double Distance(int gridCellX0, int gridCellY0,
                           int gridCellX1, int gridCellY1) const;
    /* Calculate the distance between two grid cells */
    inline double Distance(const Point2D<int>& gridCellIdx0,
                           const Point2D<int>& gridCellIdx1) const
    { return this->Distance(gridCellIdx0.mX, gridCellIdx0.mY,
                            gridCellIdx1.mX, gridCellIdx1.mY); }
    
    /* Calculate the squared distance between two grid cells */
    inline double SquaredDistance(int gridCellX0, int gridCellY0,
                                  int gridCellX1, int gridCellY1) const;
    /* Calculate the squared distance between two grid cells */
    inline double SquaredDistance(const Point2D<int>& gridCellIdx0,
                                  const Point2D<int>& gridCellIdx1) const
    { return this->SquaredDistance(gridCellIdx0.mX, gridCellIdx0.mY,
                                   gridCellIdx1.mX, gridCellIdx1.mY); }
    
    /* Convert the grid cell index to the patch index */
    inline Point2D<int> GridCellIndexToPatchIndex(
        int gridCellX, int gridCellY) const;
    /* Convert the grid cell index to the patch index */
    inline Point2D<int> GridCellIndexToPatchIndex(
        const Point2D<int>& gridCellIdx) const
    { return this->GridCellIndexToPatchIndex(gridCellIdx.mX, gridCellIdx.mY); }

    /* Convert the patch index to the grid cell index range */
    void PatchIndexToGridCellIndexRange(
        int patchIdxX, int patchIdxY,
        int& gridCellMinIdxX, int& gridCellMinIdxY,
        int& gridCellMaxIdxX, int& gridCellMaxIdxY) const;
    /* Convert the patch index to the grid cell index range */
    void PatchIndexToGridCellIndexRange(const Point2D<int>& patchIdx,
                                        Point2D<int>& gridCellMinIdx,
                                        Point2D<int>& gridCellMaxIdx) const
    { this->PatchIndexToGridCellIndexRange(
        patchIdx.mX, patchIdx.mY,
        gridCellMinIdx.mX, gridCellMinIdx.mY,
        gridCellMaxIdx.mX, gridCellMaxIdx.mY); }
    
    /* Check if the patch index is inside the map */
    inline bool PatchIsInside(int patchIdxX, int patchIdxY) const;
    /* Check if the patch index is inside the map */
    inline bool PatchIsInside(const Point2D<int>& patchIdx) const
    { return this->PatchIsInside(patchIdx.mX, patchIdx.mY); }
    
    /* Get the patch of the specified index */
    inline Patch<T>& PatchAt(int patchIdxX, int patchIdxY);
    /* Get the patch of the specified index */
    inline Patch<T>& PatchAt(const Point2D<int>& patchIdx)
    { return this->PatchAt(patchIdx.mX, patchIdx.mY); }

    /* Get the patch of the specified index */
    inline const Patch<T>& PatchAt(int patchIdxX, int patchIdxY) const;
    /* Get the patch of the specified index */
    inline const Patch<T>& PatchAt(const Point2D<int>& patchIdx) const
    { return this->PatchAt(patchIdx.mX, patchIdx.mY); }

    /* Get the map resolution (grid cell size in meters) */
    inline double MapResolution() const { return this->mMapResolution; }

    /* Get the size of the patch (in number of grid cells) */
    inline int PatchSize() const { return this->mPatchSize; }

    /* Get the number of the patches (horizontal) */
    inline int NumOfPatchesX() const { return this->mNumOfPatchesX; }
    /* Get the number of the patches (vertical) */
    inline int NumOfPatchesY() const { return this->mNumOfPatchesY; }
    
    /* Get the number of the grid cells (horizontal) */
    inline int NumOfGridCellsX() const { return this->mNumOfGridCellsX; }
    /* Get the number of the grid cells (vertical) */
    inline int NumOfGridCellsY() const { return this->mNumOfGridCellsY; }

    /* Get the size of the map in meters (horizontal) */
    inline double MapSizeX() const { return this->mMapSizeX; }
    /* Get the size of the map in meters (vertical) */
    inline double MapSizeY() const { return this->mMapSizeY; }

    /* Get the minimum position of the map in world coordinate */
    inline const Point2D<double>& MinPos() const { return this->mMinPos; }

private:
    /* Map resolution (grid cell size in meters) */
    double                      mMapResolution;
    /* Size of the patch (in number of grid cells) */
    int                         mPatchSize;
    /* Number of the patches (horizontal) */
    int                         mNumOfPatchesX;
    /* Number of the patches (vertical) */
    int                         mNumOfPatchesY;
    /* Number of the grid cells (horizontal) */
    int                         mNumOfGridCellsX;
    /* Number of the grid cells (vertical) */
    int                         mNumOfGridCellsY;
    /* Size of the map in meters (horizontal) */
    double                      mMapSizeX;
    /* Size of the map in meters (vertical) */
    double                      mMapSizeY;
    /* Minimum position of the map in world frame
     * corresponding to the origin grid cell (0, 0)
     * No relationship to pose graph nodes */
    Point2D<double>             mMinPos;
    /* Patches */
    std::unique_ptr<Patch<T>[]> mPatches;
};

/* Constructor with number of grid cells */
template <typename T>
GridMap<T>::GridMap(double mapResolution,
                    int patchSize,
                    int initialNumOfCellsX,
                    int initialNumOfCellsY,
                    const Point2D<double>& centerPos)
{
    /* Input validity checks */
    assert(mapResolution > 0.0);
    assert(patchSize > 0);
    assert(initialNumOfCellsX >= 0);
    assert(initialNumOfCellsY >= 0);

    /* Set the map resolution (size of the each grid cell) */
    this->mMapResolution = mapResolution;
    /* Set the patch size (in number of grid cells) */
    this->mPatchSize = patchSize;

    /* Map should contain at least one patch */
    initialNumOfCellsX = std::max(1, initialNumOfCellsX);
    initialNumOfCellsY = std::max(1, initialNumOfCellsY);

    this->mNumOfPatchesX = static_cast<int>(std::ceil(
        static_cast<double>(initialNumOfCellsX) /
        static_cast<double>(patchSize)));
    this->mNumOfPatchesY = static_cast<int>(std::ceil(
        static_cast<double>(initialNumOfCellsY) /
        static_cast<double>(patchSize)));
    
    assert(this->mNumOfPatchesX > 0);
    assert(this->mNumOfPatchesY > 0);

    /* Calculate the number of grid cells */
    this->mNumOfGridCellsX = this->mNumOfPatchesX * this->mPatchSize;
    this->mNumOfGridCellsY = this->mNumOfPatchesY * this->mPatchSize;
    /* Calculate the map size in meters */
    this->mMapSizeX = this->mNumOfGridCellsX * this->mMapResolution;
    this->mMapSizeY = this->mNumOfGridCellsY * this->mMapResolution;

    /* Calculate the minimum position */
    const double idxOffsetX = (this->mNumOfGridCellsX % 2 == 0) ?
        (this->mNumOfGridCellsX / 2) : (this->mNumOfGridCellsX / 2 + 0.5);
    const double idxOffsetY = (this->mNumOfGridCellsY % 2 == 0) ?
        (this->mNumOfGridCellsY / 2) : (this->mNumOfGridCellsY / 2 + 0.5);
    this->mMinPos.mX = centerPos.mX - idxOffsetX * this->mMapResolution;
    this->mMinPos.mY = centerPos.mY - idxOffsetY * this->mMapResolution;

    /* Allocate the memory for map patches */
    const int numOfPatches = this->mNumOfPatchesX * this->mNumOfPatchesY;
    this->mPatches.reset(new Patch<T>[numOfPatches]);
    assert(this->mPatches != nullptr);
}

/* Constructor with map size */
template <typename T>
GridMap<T>::GridMap(double mapResolution,
                    int patchSize,
                    double mapSizeX,
                    double mapSizeY,
                    const Point2D<double>& centerPos)
{
    /* Input validity checks */
    assert(mapResolution > 0.0);
    assert(patchSize > 0);
    assert(mapSizeX >= 0.0);
    assert(mapSizeY >= 0.0);

    /* Set the map resolution (size of the each grid cell) */
    this->mMapResolution = mapResolution;
    /* Set the patch size (in number of grid cells) */
    this->mPatchSize = patchSize;

    /* Modify the map size so that the map contains at least one patch */
    const double patchSizeInMeters = patchSize * this->mMapResolution;
    mapSizeX = std::max(patchSizeInMeters, mapSizeX);
    mapSizeY = std::max(patchSizeInMeters, mapSizeY);

    /* Calculate and validate the number of patches */
    this->mNumOfPatchesX = static_cast<int>(std::ceil(
        mapSizeX / patchSizeInMeters));
    this->mNumOfPatchesY = static_cast<int>(std::ceil(
        mapSizeY / patchSizeInMeters));
    
    assert(this->mNumOfPatchesX > 0);
    assert(this->mNumOfPatchesY > 0);

    /* Calculate the number of grid cells */
    this->mNumOfGridCellsX = this->mNumOfPatchesX * this->mPatchSize;
    this->mNumOfGridCellsY = this->mNumOfPatchesY * this->mPatchSize;
    /* Calculate the map size in meters */
    this->mMapSizeX = this->mNumOfGridCellsX * this->mMapResolution;
    this->mMapSizeY = this->mNumOfGridCellsY * this->mMapResolution;

    /* Calculate the minimum position
     * which corresponds to the grid cell (0, 0) */
    const double idxOffsetX = (this->mNumOfGridCellsX % 2 == 0) ?
        (this->mNumOfGridCellsX / 2) : (this->mNumOfGridCellsX / 2 + 0.5);
    const double idxOffsetY = (this->mNumOfGridCellsY % 2 == 0) ?
        (this->mNumOfGridCellsY / 2) : (this->mNumOfGridCellsY / 2 + 0.5);
    this->mMinPos.mX = centerPos.mX - idxOffsetX * this->mMapResolution;
    this->mMinPos.mY = centerPos.mY - idxOffsetY * this->mMapResolution;

    /* Allocate the memory for map patches (not grid cells) */
    const int numOfPatches = this->mNumOfPatchesX * this->mNumOfPatchesY;
    this->mPatches.reset(new Patch<T>[numOfPatches]);
    assert(this->mPatches != nullptr);
}

/* Constructor with positions */
template <typename T>
GridMap<T>::GridMap(double mapResolution,
                    int patchSize,
                    double minX,
                    double minY,
                    double maxX,
                    double maxY)
{
    /* Input validity checks */
    assert(mapResolution > 0.0);
    assert(patchSize > 0);
    assert(minX <= maxX);
    assert(minY <= maxY);

    /* Set the map resolution (size of the each grid cell) */
    this->mMapResolution = mapResolution;
    /* Set the patch size (in number of grid cells) */
    this->mPatchSize = patchSize;

    /* Map should contain at least one patch */
    const double patchSizeInMeters = patchSize * mapResolution;
    double mapSizeX = std::max(patchSizeInMeters, maxX - minX);
    double mapSizeY = std::max(patchSizeInMeters, maxY - minY);
    
    /* Calculate the number of patches */
    this->mNumOfPatchesX = static_cast<int>(std::ceil(
        mapSizeX / patchSizeInMeters));
    this->mNumOfPatchesY = static_cast<int>(std::ceil(
        mapSizeY / patchSizeInMeters));
    
    assert(this->mNumOfPatchesX > 0);
    assert(this->mNumOfPatchesY > 0);

    /* Calculate the number of grid cells */
    this->mNumOfGridCellsX = this->mNumOfPatchesX * this->mPatchSize;
    this->mNumOfGridCellsY = this->mNumOfPatchesY * this->mPatchSize;
    /* Calculate the map size in meters */
    this->mMapSizeX = this->mNumOfGridCellsX * this->mMapResolution;
    this->mMapSizeY = this->mNumOfGridCellsY * this->mMapResolution;

    /* Set the minimum position */
    this->mMinPos.mX = minX;
    this->mMinPos.mY = minY;

    /* Allocate the memory for map patches */
    const int numOfPatches = this->mNumOfPatchesX * this->mNumOfPatchesY;
    this->mPatches.reset(new Patch<T>[numOfPatches]);
    assert(this->mPatches != nullptr);
}

/* Move constructor */
template <typename T>
GridMap<T>::GridMap(GridMap<T>&& other) noexcept :
    mMapResolution(other.mMapResolution),
    mPatchSize(other.mPatchSize),
    mNumOfPatchesX(other.mNumOfPatchesX),
    mNumOfPatchesY(other.mNumOfPatchesY),
    mNumOfGridCellsX(other.mNumOfGridCellsX),
    mNumOfGridCellsY(other.mNumOfGridCellsY),
    mMapSizeX(other.mMapSizeX),
    mMapSizeY(other.mMapSizeY),
    mMinPos(other.mMinPos),
    mPatches(std::move(other.mPatches))
{
}

/* Move assignment operator */
template <typename T>
GridMap<T>& GridMap<T>::operator=(GridMap<T>&& other) noexcept
{
    if (this == &other)
        return *this;
    
    this->mMapResolution = other.mMapResolution;
    this->mPatchSize = other.mPatchSize;
    this->mNumOfPatchesX = other.mNumOfPatchesX;
    this->mNumOfPatchesY = other.mNumOfPatchesY;
    this->mNumOfGridCellsX = other.mNumOfGridCellsX;
    this->mNumOfGridCellsY = other.mNumOfGridCellsY;
    this->mMapSizeX = other.mMapSizeX;
    this->mMapSizeY = other.mMapSizeY;
    this->mMinPos = other.mMinPos;
    this->mPatches = std::move(other.mPatches);

    return *this;
}

/* Resize the map */
template <typename T>
void GridMap<T>::Resize(double minX, double minY,
                        double maxX, double maxY)
{
    assert(minX <= maxX);
    assert(minY <= maxY);

    /* Calculate the grid cell index (can be negative) */
    const Point2D<int> gridCellIdxMin =
        this->WorldCoordinateToGridCellIndex(minX, minY);
    const Point2D<int> gridCellIdxMax =
        this->WorldCoordinateToGridCellIndex(maxX, maxY);
    
    /* Calculate the patch index (can be negative) */
    const Point2D<int> patchIdxMin =
        this->GridCellIndexToPatchIndex(gridCellIdxMin);
    const Point2D<int> patchIdxMax =
        this->GridCellIndexToPatchIndex(gridCellIdxMax);
    
    /* Calculate the number of patches */
    const int numOfPatchesX = std::max(1, patchIdxMax.mX - patchIdxMin.mX + 1);
    const int numOfPatchesY = std::max(1, patchIdxMax.mY - patchIdxMin.mY + 1);

    /* Re-allocate patches */
    std::unique_ptr<Patch<T>[]> oldPatches = std::move(this->mPatches);
    this->mPatches.reset(new Patch<T>[numOfPatchesX * numOfPatchesY]);
    assert(this->mPatches != nullptr);

    /* Copy patches */
    const int x0 = std::max(0, patchIdxMin.mX);
    const int y0 = std::max(0, patchIdxMin.mY);
    const int x1 = std::min(this->mNumOfPatchesX, patchIdxMax.mX + 1);
    const int y1 = std::min(this->mNumOfPatchesY, patchIdxMax.mY + 1);

    for (int y = y0; y < y1; ++y) {
        for (int x = x0; x < x1; ++x) {
            const int patchIdx = (y - patchIdxMin.mY) * numOfPatchesX +
                                 (x - patchIdxMin.mX);
            const int oldPatchIdx = y * this->mNumOfPatchesX + x;
            this->mPatches[patchIdx] = std::move(oldPatches[oldPatchIdx]);
        }
    }

    /* Update map parameters */
    this->mNumOfPatchesX = numOfPatchesX;
    this->mNumOfPatchesY = numOfPatchesY;
    this->mNumOfGridCellsX = this->mNumOfPatchesX * this->mPatchSize;
    this->mNumOfGridCellsY = this->mNumOfPatchesY * this->mPatchSize;
    this->mMapSizeX = this->mNumOfGridCellsX * this->mMapResolution;
    this->mMapSizeY = this->mNumOfGridCellsY * this->mMapResolution;

    this->mMinPos.mX +=
        (patchIdxMin.mX * this->mPatchSize) * this->mMapResolution;
    this->mMinPos.mY +=
        (patchIdxMin.mY * this->mPatchSize) * this->mMapResolution;
}

/* Expand the map if necessary */
template <typename T>
void GridMap<T>::Expand(double minX, double minY,
                        double maxX, double maxY,
                        double enlargeStep)
{
    assert(minX <= maxX);
    assert(minY <= maxY);

    if (this->IsInside(minX, minY) && this->IsInside(maxX, maxY))
        return;
    
    Point2D<double> minPos = this->GridCellIndexToWorldCoordinate(0, 0);
    Point2D<double> maxPos = this->GridCellIndexToWorldCoordinate(
        this->mNumOfGridCellsX, this->mNumOfGridCellsY);

    minPos.mX = (minX < minPos.mX) ? minX - enlargeStep : minPos.mX;
    minPos.mY = (minY < minPos.mY) ? minY - enlargeStep : minPos.mY;
    maxPos.mX = (maxX > maxPos.mX) ? maxX + enlargeStep : maxPos.mX;
    maxPos.mY = (maxY > maxPos.mY) ? maxY + enlargeStep : maxPos.mY;

    /* Expand the map if necessary */
    this->Resize(minPos.mX, minPos.mY, maxPos.mX, maxPos.mY);
}

/* Reset occupancy probability values of all grid cells */
template <typename T>
void GridMap<T>::Reset()
{
    assert(this->mNumOfPatchesX > 0);
    assert(this->mNumOfPatchesY > 0);
    assert(this->mPatches != nullptr);

    /* Reset all patches */
    for (int patchIdxY = 0; patchIdxY < this->mNumOfPatchesY; ++patchIdxY)
        for (int patchIdxX = 0; patchIdxX < this->mNumOfPatchesX; ++patchIdxX)
            this->PatchAt(patchIdxX, patchIdxY).Reset();
}

/* Check if the grid cell index is inside the map */
template <typename T>
bool GridMap<T>::IsInside(int gridCellX, int gridCellY) const
{
    return (gridCellX >= 0 && gridCellX < this->mNumOfGridCellsX) &&
           (gridCellY >= 0 && gridCellY < this->mNumOfGridCellsY);
}

/* Convert the grid cell index into the point in world frame
 * The returned point is the bottom-left of the grid cell */
template <typename T>
Point2D<double> GridMap<T>::GridCellIndexToWorldCoordinate(
    int gridCellIdxX, int gridCellIdxY) const
{
    const double mapX =
        this->mMinPos.mX + this->mMapResolution * gridCellIdxX;
    const double mapY =
        this->mMinPos.mY + this->mMapResolution * gridCellIdxY;

    return Point2D<double> { mapX, mapY };
}

/* Convert the point in world frame to the grid cell index */
template <typename T>
Point2D<int> GridMap<T>::WorldCoordinateToGridCellIndex(
    double mapX, double mapY) const
{
    const int cellIdxX = static_cast<int>(std::floor(
        (mapX - this->mMinPos.mX) / this->mMapResolution));
    const int cellIdxY = static_cast<int>(std::floor(
        (mapY - this->mMinPos.mY) / this->mMapResolution));
    
    return Point2D<int> { cellIdxX, cellIdxY };
}

/* Get the grid cell of the specified index */
template <typename T>
typename GridMap<T>::GridCellType& GridMap<T>::GridCellAt(
    int gridCellX, int gridCellY)
{
    assert(this->IsInside(gridCellX, gridCellY));

    const Point2D<int> patchIdx =
        this->GridCellIndexToPatchIndex(gridCellX, gridCellY);
    Patch<T>& patch = this->PatchAt(patchIdx);

    /* Allocate patch if necessary */
    if (!patch.IsAllocated())
        patch.Allocate(this->mPatchSize);
    
    return patch.At(gridCellX % this->mPatchSize,
                    gridCellY % this->mPatchSize);
}

/* Get the grid cell of the specified index */
template <typename T>
const typename GridMap<T>::GridCellType& GridMap<T>::GridCellAt(
    int gridCellX, int gridCellY) const
{
    assert(this->IsInside(gridCellX, gridCellY));

    const Point2D<int> patchIdx =
        this->GridCellIndexToPatchIndex(gridCellX, gridCellY);
    const Patch<T>& patch = this->PatchAt(patchIdx);

    return patch.At(gridCellX % this->mPatchSize,
                    gridCellY % this->mPatchSize);
}

/* Get the occupancy probability value of the specified grid cell */
template <typename T>
typename GridMap<T>::ValueType GridMap<T>::Value(
    int gridCellX, int gridCellY) const
{
    assert(this->IsInside(gridCellX, gridCellY));

    const Point2D<int> patchIdx =
        this->GridCellIndexToPatchIndex(gridCellX, gridCellY);
    const Patch<T>& patch = this->PatchAt(patchIdx);

    return patch.Value(gridCellX % this->mPatchSize,
                       gridCellY % this->mPatchSize);
}

/* Get the occupancy probability value of the specified grid cell
 * The default value is returned if the specified grid cell is
 * out of bounds or is not yet allocated */
template <typename T>
typename GridMap<T>::ValueType GridMap<T>::Value(
    int gridCellX, int gridCellY, ValueType defaultValue) const
{
    if (!this->IsInside(gridCellX, gridCellY))
        return defaultValue;
    
    const Point2D<int> patchIdx =
        this->GridCellIndexToPatchIndex(gridCellX, gridCellY);
    const Patch<T>& patch = this->PatchAt(patchIdx);

    return patch.Value(gridCellX % this->mPatchSize,
                       gridCellY % this->mPatchSize,
                       defaultValue);
}

/* Update the occupancy probability value of the specified grid cell */
template <typename T>
void GridMap<T>::Update(int gridCellX, int gridCellY,
                        const ObservationType& latestObservation)
{
    this->GridCellAt(gridCellX, gridCellY).Update(latestObservation);
}

/* Calculate the distance between two grid cells */
template <typename T>
double GridMap<T>::Distance(int gridCellX0, int gridCellY0,
                            int gridCellX1, int gridCellY1) const
{
    return std::sqrt(
        std::pow((gridCellX1 - gridCellX0) * this->mMapResolution, 2.0) +
        std::pow((gridCellY1 - gridCellY0) * this->mMapResolution, 2.0));
}

/* Calculate the squared distance between two grid cells */
template <typename T>
double GridMap<T>::SquaredDistance(int gridCellX0, int gridCellY0,
                                   int gridCellX1, int gridCellY1) const
{
    return std::pow((gridCellX1 - gridCellX0) * this->mMapResolution, 2.0) +
           std::pow((gridCellY1 - gridCellY0) * this->mMapResolution, 2.0);
}

/* Convert the grid cell index to the patch index */
template <typename T>
Point2D<int> GridMap<T>::GridCellIndexToPatchIndex(
    int gridCellX, int gridCellY) const
{
    return Point2D<int> {
        (gridCellX < 0) ? (gridCellX / this->mPatchSize - 1) :
                          (gridCellX / this->mPatchSize),
        (gridCellY < 0) ? (gridCellY / this->mPatchSize - 1) :
                          (gridCellY / this->mPatchSize) };
}

/* Convert the patch index to the grid cell index range */
template <typename T>
void GridMap<T>::PatchIndexToGridCellIndexRange(
    int patchIdxX, int patchIdxY,
    int& gridCellMinIdxX, int& gridCellMinIdxY,
    int& gridCellMaxIdxX, int& gridCellMaxIdxY) const
{
    gridCellMinIdxX = patchIdxX * this->mPatchSize;
    gridCellMinIdxY = patchIdxY * this->mPatchSize;
    gridCellMaxIdxX = gridCellMinIdxX + this->mPatchSize;
    gridCellMaxIdxY = gridCellMinIdxY + this->mPatchSize;
}

/* Check if the patch index is inside the map */
template <typename T>
bool GridMap<T>::PatchIsInside(int patchIdxX, int patchIdxY) const
{
    return (patchIdxX >= 0 && patchIdxX < this->mNumOfPatchesX) &&
           (patchIdxY >= 0 && patchIdxY < this->mNumOfPatchesY);
}

/* Get the patch of the specified index */
template <typename T>
Patch<T>& GridMap<T>::PatchAt(int patchIdxX, int patchIdxY)
{
    assert(this->PatchIsInside(patchIdxX, patchIdxY));
    return this->mPatches[patchIdxY * this->mNumOfPatchesX + patchIdxX];
}

/* Get the patch of the specified index */
template <typename T>
const Patch<T>& GridMap<T>::PatchAt(int patchIdxX, int patchIdxY) const
{
    assert(this->PatchIsInside(patchIdxX, patchIdxY));
    return this->mPatches[patchIdxY * this->mNumOfPatchesX + patchIdxX];
}

} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_GRID_MAP_GRID_MAP_HPP */
