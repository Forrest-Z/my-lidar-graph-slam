
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
#include "my_lidar_graph_slam/grid_map/grid_map_base.hpp"
#include "my_lidar_graph_slam/grid_map/grid_map_patch.hpp"

namespace MyLidarGraphSlam {

template <typename T>
class GridMap final : public GridMapBase<typename T::ValueType>
{
public:
    /* Type definitions */
    using GridCellType = T;
    using PatchType = Patch<T>;
    using ValueType = typename T::ValueType;
    using ObservationType = typename T::ObservationType;

    /* Constructor to create an empty grid map */
    GridMap();

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

    /* Constructor with internal parameters */
    GridMap(double mapResolution,
            int patchSize,
            int numOfPatchesX,
            int numOfPatchesY,
            double minX,
            double minY);
    
    /* Destructor */
    ~GridMap() = default;

    /* Copy constructor */
    GridMap(const GridMap& other);
    /* Copy assignment operator */
    GridMap& operator=(const GridMap& other);
    /* Move constructor */
    GridMap(GridMap&& other) noexcept;
    /* Move assignment operator */
    GridMap& operator=(GridMap&& other) noexcept;

    /* Create a new grid map that has the same size as 'gridMap' */
    template <typename U>
    static GridMap CreateSameSizeMap(const GridMap<U>& gridMap);

public:
    /* Resize the map */
    void Resize(double minX, double minY,
                double maxX, double maxY);
    /* Expand the map if necessary */
    void Expand(double minX, double minY,
                double maxX, double maxY,
                double enlargeStep = 5.0);
    
    /* Reset occupancy probability values of all grid cells */
    void Reset();

    /* Get the unknown occupancy probability value */
    ValueType UnknownValue() const override { return GridCellType::Unknown; }
 
    /* Check if the grid cell index is inside the map */
    bool IsInside(int idxX, int idxY) const override;
    /* Check if the grid cell index is inside the map */
    bool IsInside(const Point2D<int>& gridCellIdx) const override
    { return this->IsInside(gridCellIdx.mX, gridCellIdx.mY); }

    /* Check if the point in world frame is inside the map */
    bool IsInside(double mapX, double mapY) const override
    { return this->IsInside(
        this->WorldCoordinateToGridCellIndex(mapX, mapY)); }
    /* Check if the point in world frame is inside the map */
    bool IsInside(const Point2D<double>& mapPos) const override
    { return this->IsInside(mapPos.mX, mapPos.mY); }

    /* Check if the grid cell is allocated on the heap */
    bool IsAllocated(int idxX, int idxY) const override
    { return this->PatchIsAllocated(
        this->GridCellIndexToPatchIndex(idxX, idxY)); }
    /* Check if the grid cell is allocated on the heap */
    bool IsAllocated(const Point2D<int>& gridCellIdx) const override
    { return this->IsAllocated(gridCellIdx.mX, gridCellIdx.mY); }

    /* Convert the grid cell index into the point in world frame
     * The returned point is the bottom-left of the grid cell */
    Point2D<double> GridCellIndexToWorldCoordinate(
        int idxX, int idxY) const override;
    /* Convert the grid cell index into the point in world frame
     * The returned point is the bottom-left of the grid cell */
    Point2D<double> GridCellIndexToWorldCoordinate(
        const Point2D<int>& gridCellIdx) const override
    { return this->GridCellIndexToWorldCoordinate(
        gridCellIdx.mX, gridCellIdx.mY); }
    
    /* Convert the point in world frame to the grid cell index */
    Point2D<int> WorldCoordinateToGridCellIndex(
        double mapX, double mapY) const override;
    /* Convert the point in world frame to the grid cell index */
    Point2D<int> WorldCoordinateToGridCellIndex(
        const Point2D<double>& mapPos) const override
    { return this->WorldCoordinateToGridCellIndex(mapPos.mX, mapPos.mY); }

    /* Convert the point in world frame to the
     * floating-point grid cell index */
    Point2D<double> WorldCoordinateToGridCellIndexFloat(
        double mapX, double mapY) const override;
    /* Convert the point in world frame to the
     * floating-point grid cell index */
    Point2D<double> WorldCoordinateToGridCellIndexFloat(
        const Point2D<double>& mapPos) const override
    { return this->WorldCoordinateToGridCellIndexFloat(mapPos.mX, mapPos.mY); }

    /* Get the grid cell of the specified index */
    GridCellType& GridCellAt(int idxX, int idxY);
    /* Get the grid cell of the specified index */
    GridCellType& GridCellAt(const Point2D<int>& gridCellIdx)
    { return this->GridCellAt(gridCellIdx.mX, gridCellIdx.mY); }

    /* Get the grid cell of the specified index */
    const GridCellType& GridCellAt(int idxX, int idxY) const;
    /* Get the grid cell of the specified index */
    const GridCellType& GridCellAt(const Point2D<int>& gridCellIdx) const
    { return this->GridCellAt(gridCellIdx.mX, gridCellIdx.mY); }

    /* Get the occupancy probability value of the specified grid cell */
    ValueType Value(int idxX, int idxY) const override;
    /* Get the occupancy probability value of the specified grid cell */
    ValueType Value(const Point2D<int>& gridCellIdx) const override
    { return this->Value(gridCellIdx.mX, gridCellIdx.mY); }

    /* Get the occupancy probability value of the specified grid cell
     * The default value is returned if the specified grid cell is
     * out of bounds or is not yet allocated */
    ValueType Value(int idxX, int idxY, ValueType defaultVal) const override;
    /* Get the occupancy probability value of the specified grid cell */
    ValueType Value(const Point2D<int>& gridCellIdx,
                    ValueType defaultVal) const override
    { return this->Value(gridCellIdx.mX, gridCellIdx.mY, defaultVal); }

    /* Update the occupancy probability value of the specified grid cell */
    void Update(int idxX, int idxY, const ObservationType& updateVal);
    /* Update the occupancy probability value of the specified grid cell */
    void Update(const Point2D<int>& gridCellIdx,
                const ObservationType& updateVal)
    { this->Update(gridCellIdx.mX, gridCellIdx.mY, updateVal); }

    /* Update the occupancy probability value of the specified point */
    void Update(double mapX, double mapY, const ObservationType& updateVal)
    { this->Update(this->WorldCoordinateToGridCellIndex(mapX, mapY),
                   updateVal); }
    /* Update the occupancy probability value of the specified point */
    void Update(const Point2D<double>& mapPos,
                const ObservationType& updateVal)
    { this->Update(mapPos.mX, mapPos.mY, updateVal); }

    /* Calculate the distance between two grid cells */
    double Distance(int idxX0, int idxY0,
                    int idxX1, int idxY1) const override;
    /* Calculate the distance between two grid cells */
    double Distance(const Point2D<int>& gridCellIdx0,
                    const Point2D<int>& gridCellIdx1) const override
    { return this->Distance(gridCellIdx0.mX, gridCellIdx0.mY,
                            gridCellIdx1.mX, gridCellIdx1.mY); }
    
    /* Calculate the squared distance between two grid cells */
    double SquaredDistance(int idxX0, int idxY0,
                           int idxX1, int idxY1) const override;
    /* Calculate the squared distance between two grid cells */
    double SquaredDistance(const Point2D<int>& gridCellIdx0,
                           const Point2D<int>& gridCellIdx1) const override
    { return this->SquaredDistance(gridCellIdx0.mX, gridCellIdx0.mY,
                                   gridCellIdx1.mX, gridCellIdx1.mY); }
    
    /* Convert the grid cell index to the patch index */
    Point2D<int> GridCellIndexToPatchIndex(int idxX, int idxY) const;
    /* Convert the grid cell index to the patch index */
    Point2D<int> GridCellIndexToPatchIndex(
        const Point2D<int>& gridCellIdx) const
    { return this->GridCellIndexToPatchIndex(gridCellIdx.mX, gridCellIdx.mY); }

    /* Convert the patch index to the grid cell index range
     * patch (patchIdxX, patchIdxY) corresponds to the square grid cells
     * [minIdxX, maxIdxX) * [minIdxY, maxIdxY) */
    void PatchIndexToGridCellIndexRange(
        int patchIdxX, int patchIdxY,
        int& minIdxX, int& minIdxY,
        int& maxIdxX, int& maxIdxY) const;
    /* Convert the patch index to the grid cell index range */
    void PatchIndexToGridCellIndexRange(const Point2D<int>& patchIdx,
                                        Point2D<int>& minIdx,
                                        Point2D<int>& maxIdx) const
    { this->PatchIndexToGridCellIndexRange(
        patchIdx.mX, patchIdx.mY,
        minIdx.mX, minIdx.mY,
        maxIdx.mX, maxIdx.mY); }
    
    /* Check if the patch is inside the map */
    bool PatchIsInside(int patchIdxX, int patchIdxY) const;
    /* Check if the patch is inside the map */
    bool PatchIsInside(const Point2D<int>& patchIdx) const
    { return this->PatchIsInside(patchIdx.mX, patchIdx.mY); }

    /* Check if the patch is allocated on the heap */
    bool PatchIsAllocated(int patchIdxX, int patchIdxY) const;
    /* Check if the patch is allocated on the heap */
    bool PatchIsAllocated(const Point2D<int>& patchIdx) const
    { return this->PatchIsAllocated(patchIdx.mX, patchIdx.mY); }

    /* Get the patch of the specified index */
    Patch<T>& PatchAt(int patchIdxX, int patchIdxY);
    /* Get the patch of the specified index */
    Patch<T>& PatchAt(const Point2D<int>& patchIdx)
    { return this->PatchAt(patchIdx.mX, patchIdx.mY); }

    /* Get the patch of the specified index */
    const Patch<T>& PatchAt(int patchIdxX, int patchIdxY) const;
    /* Get the patch of the specified index */
    const Patch<T>& PatchAt(const Point2D<int>& patchIdx) const
    { return this->PatchAt(patchIdx.mX, patchIdx.mY); }

    /* Get the map resolution (grid cell size in meters) */
    inline double Resolution() const override
    { return this->mResolution; }

    /* Get the size of the patch (in number of grid cells) */
    inline int PatchSize() const { return this->mPatchSize; }

    /* Get the number of the patches (horizontal) */
    inline int NumOfPatchesX() const { return this->mNumOfPatchesX; }
    /* Get the number of the patches (vertical) */
    inline int NumOfPatchesY() const { return this->mNumOfPatchesY; }
    
    /* Get the number of the grid cells (horizontal) */
    inline int NumOfGridCellsX() const override
    { return this->mNumOfGridCellsX; }
    /* Get the number of the grid cells (vertical) */
    inline int NumOfGridCellsY() const override
    { return this->mNumOfGridCellsY; }

    /* Get the size of the map in meters (horizontal) */
    inline double MapSizeX() const override { return this->mMapSizeX; }
    /* Get the size of the map in meters (vertical) */
    inline double MapSizeY() const override { return this->mMapSizeY; }

    /* Get the minimum position of the map in world coordinate */
    inline const Point2D<double>& MinPos() const override
    { return this->mMinPos; }

private:
    /* Map resolution (grid cell size in meters) */
    double                      mResolution;
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

/* Constructor to create an empty grid map */
template <typename T>
GridMap<T>::GridMap() :
    mResolution(0.0),
    mPatchSize(0),
    mNumOfPatchesX(0),
    mNumOfPatchesY(0),
    mNumOfGridCellsX(0),
    mNumOfGridCellsY(0),
    mMapSizeX(0.0),
    mMapSizeY(0.0),
    mMinPos(0.0, 0.0),
    mPatches(nullptr)
{
}

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
    this->mResolution = mapResolution;
    /* Set the patch size (in number of grid cells) */
    this->mPatchSize = patchSize;

    /* Empty grid map with no patch is allowed */
    initialNumOfCellsX = std::max(0, initialNumOfCellsX);
    initialNumOfCellsY = std::max(0, initialNumOfCellsY);

    this->mNumOfPatchesX = static_cast<int>(std::ceil(
        static_cast<double>(initialNumOfCellsX) /
        static_cast<double>(patchSize)));
    this->mNumOfPatchesY = static_cast<int>(std::ceil(
        static_cast<double>(initialNumOfCellsY) /
        static_cast<double>(patchSize)));
    
    assert(this->mNumOfPatchesX >= 0);
    assert(this->mNumOfPatchesY >= 0);

    /* Calculate the number of grid cells */
    this->mNumOfGridCellsX = this->mNumOfPatchesX * this->mPatchSize;
    this->mNumOfGridCellsY = this->mNumOfPatchesY * this->mPatchSize;
    /* Calculate the map size in meters */
    this->mMapSizeX = this->mNumOfGridCellsX * this->mResolution;
    this->mMapSizeY = this->mNumOfGridCellsY * this->mResolution;

    /* Calculate the minimum position */
    const double idxOffsetX = (this->mNumOfGridCellsX % 2 == 0) ?
        (this->mNumOfGridCellsX / 2) : (this->mNumOfGridCellsX / 2 + 0.5);
    const double idxOffsetY = (this->mNumOfGridCellsY % 2 == 0) ?
        (this->mNumOfGridCellsY / 2) : (this->mNumOfGridCellsY / 2 + 0.5);
    this->mMinPos.mX = centerPos.mX - idxOffsetX * this->mResolution;
    this->mMinPos.mY = centerPos.mY - idxOffsetY * this->mResolution;

    /* Allocate the memory for map patches if necessary */
    const int numOfPatches = this->mNumOfPatchesX * this->mNumOfPatchesY;

    if (numOfPatches > 0)
        this->mPatches.reset(new Patch<T>[numOfPatches]);
    else
        this->mPatches.reset(nullptr);
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
    this->mResolution = mapResolution;
    /* Set the patch size (in number of grid cells) */
    this->mPatchSize = patchSize;

    /* Empty grid map with no patch is allowed */
    mapSizeX = std::max(0.0, mapSizeX);
    mapSizeY = std::max(0.0, mapSizeY);

    /* Calculate and validate the number of patches */
    const double patchSizeInMeters = patchSize * this->mResolution;
    this->mNumOfPatchesX = static_cast<int>(std::ceil(
        mapSizeX / patchSizeInMeters));
    this->mNumOfPatchesY = static_cast<int>(std::ceil(
        mapSizeY / patchSizeInMeters));
    
    assert(this->mNumOfPatchesX >= 0);
    assert(this->mNumOfPatchesY >= 0);

    /* Calculate the number of grid cells */
    this->mNumOfGridCellsX = this->mNumOfPatchesX * this->mPatchSize;
    this->mNumOfGridCellsY = this->mNumOfPatchesY * this->mPatchSize;
    /* Calculate the map size in meters */
    this->mMapSizeX = this->mNumOfGridCellsX * this->mResolution;
    this->mMapSizeY = this->mNumOfGridCellsY * this->mResolution;

    /* Calculate the minimum position
     * which corresponds to the grid cell (0, 0) */
    const double idxOffsetX = (this->mNumOfGridCellsX % 2 == 0) ?
        (this->mNumOfGridCellsX / 2) : (this->mNumOfGridCellsX / 2 + 0.5);
    const double idxOffsetY = (this->mNumOfGridCellsY % 2 == 0) ?
        (this->mNumOfGridCellsY / 2) : (this->mNumOfGridCellsY / 2 + 0.5);
    this->mMinPos.mX = centerPos.mX - idxOffsetX * this->mResolution;
    this->mMinPos.mY = centerPos.mY - idxOffsetY * this->mResolution;

    /* Allocate the memory for map patches (not grid cells) */
    const int numOfPatches = this->mNumOfPatchesX * this->mNumOfPatchesY;

    if (numOfPatches > 0)
        this->mPatches.reset(new Patch<T>[numOfPatches]);
    else
        this->mPatches.reset(nullptr);
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
    this->mResolution = mapResolution;
    /* Set the patch size (in number of grid cells) */
    this->mPatchSize = patchSize;

    /* Empty grid map with no patch is allowed */
    const double mapSizeX = std::max(0.0, maxX - minX);
    const double mapSizeY = std::max(0.0, maxY - minY);

    /* Calculate the number of patches */
    const double patchSizeInMeters = patchSize * this->mResolution;
    this->mNumOfPatchesX = static_cast<int>(std::ceil(
        mapSizeX / patchSizeInMeters));
    this->mNumOfPatchesY = static_cast<int>(std::ceil(
        mapSizeY / patchSizeInMeters));

    assert(this->mNumOfPatchesX >= 0);
    assert(this->mNumOfPatchesY >= 0);

    /* Calculate the number of grid cells */
    this->mNumOfGridCellsX = this->mNumOfPatchesX * this->mPatchSize;
    this->mNumOfGridCellsY = this->mNumOfPatchesY * this->mPatchSize;
    /* Calculate the map size in meters */
    this->mMapSizeX = this->mNumOfGridCellsX * this->mResolution;
    this->mMapSizeY = this->mNumOfGridCellsY * this->mResolution;

    /* Set the minimum position */
    this->mMinPos.mX = minX;
    this->mMinPos.mY = minY;

    /* Allocate the memory for map patches */
    const int numOfPatches = this->mNumOfPatchesX * this->mNumOfPatchesY;

    if (numOfPatches > 0)
        this->mPatches.reset(new Patch<T>[numOfPatches]);
    else
        this->mPatches.reset(nullptr);
}

/* Constructor with internal parameters */
template <typename T>
GridMap<T>::GridMap(double mapResolution,
                    int patchSize,
                    int numOfPatchesX,
                    int numOfPatchesY,
                    double minX,
                    double minY)
{
    assert(mapResolution > 0.0);
    assert(patchSize > 0);
    assert(numOfPatchesX >= 0);
    assert(numOfPatchesY >= 0);

    /* Set the map resolution */
    this->mResolution = mapResolution;
    /* Set the patch size */
    this->mPatchSize = patchSize;

    /* Set the number of patches */
    this->mNumOfPatchesX = numOfPatchesX;
    this->mNumOfPatchesY = numOfPatchesY;
    /* Set the number of grid cells */
    this->mNumOfGridCellsX = this->mNumOfPatchesX * this->mPatchSize;
    this->mNumOfGridCellsY = this->mNumOfPatchesY * this->mPatchSize;
    /* Set the map size in meters */
    this->mMapSizeX = this->mNumOfGridCellsX * this->mResolution;
    this->mMapSizeY = this->mNumOfGridCellsY * this->mResolution;

    /* Set the minimum position */
    this->mMinPos.mX = minX;
    this->mMinPos.mY = minY;

    /* Allocate the memory for patches */
    const int numOfPatches = this->mNumOfPatchesX * this->mNumOfPatchesY;

    if (numOfPatches > 0)
        this->mPatches.reset(new Patch<T>[numOfPatches]);
    else
        this->mPatches.reset(nullptr);
}

/* Copy constructor */
template <typename T>
GridMap<T>::GridMap(const GridMap<T>& other) :
    mResolution(0.0),
    mPatchSize(0),
    mNumOfPatchesX(0),
    mNumOfPatchesY(0),
    mNumOfGridCellsX(0),
    mNumOfGridCellsY(0),
    mMapSizeX(0.0),
    mMapSizeY(0.0),
    mMinPos(0.0, 0.0),
    mPatches(nullptr)
{
    /* Just call the copy assignment operator */
    *this = other;
}

/* Copy assignment operator */
template <typename T>
GridMap<T>& GridMap<T>::operator=(const GridMap<T>& other)
{
    if (this == &other)
        return *this;

    /* Ensure that the map size (in the number of patches) is valid */
    assert(other.mNumOfPatchesX >= 0);
    assert(other.mNumOfPatchesY >= 0);

    const int numOfPatches = other.mNumOfPatchesX * other.mNumOfPatchesY;
    assert((numOfPatches > 0 && other.mPatches != nullptr) ||
           (numOfPatches == 0 && other.mPatches == nullptr));

    /* Copy the grid map parameters */
    this->mResolution = other.mResolution;
    this->mPatchSize = other.mPatchSize;
    this->mNumOfPatchesX = other.mNumOfPatchesX;
    this->mNumOfPatchesY = other.mNumOfPatchesY;
    this->mNumOfGridCellsX = other.mNumOfGridCellsX;
    this->mNumOfGridCellsY = other.mNumOfGridCellsY;
    this->mMapSizeX = other.mMapSizeX;
    this->mMapSizeY = other.mMapSizeY;
    this->mMinPos = other.mMinPos;

    /* Allocate new patches and then copy the values */
    if (numOfPatches > 0) {
        this->mPatches.reset(new Patch<T>[numOfPatches]);
        assert(this->mPatches != nullptr);
        std::copy_n(other.mPatches.get(), numOfPatches, this->mPatches.get());
    } else {
        this->mPatches.reset(nullptr);
    }

    return *this;
}

/* Move constructor */
template <typename T>
GridMap<T>::GridMap(GridMap<T>&& other) noexcept :
    mResolution(other.mResolution),
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
    
    this->mResolution = other.mResolution;
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

/* Create a new grid map that has the same size as 'gridMap' */
template <typename T>
template <typename U>
GridMap<T> GridMap<T>::CreateSameSizeMap(const GridMap<U>& gridMap)
{
    /* Construct a new grid map with internal parameters */
    return GridMap<T> { gridMap.Resolution(), gridMap.PatchSize(),
                        gridMap.NumOfPatchesX(), gridMap.NumOfPatchesY(),
                        gridMap.MinPos().mX, gridMap.MinPos().mY };
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
    const int numOfPatchesX = std::max(0, patchIdxMax.mX - patchIdxMin.mX + 1);
    const int numOfPatchesY = std::max(0, patchIdxMax.mY - patchIdxMin.mY + 1);
    const int numOfPatches = numOfPatchesX * numOfPatchesY;

    /* Re-allocate patches */
    std::unique_ptr<Patch<T>[]> oldPatches = std::move(this->mPatches);

    if (numOfPatches > 0)
        this->mPatches.reset(new Patch<T>[numOfPatchesX * numOfPatchesY]);
    else
        this->mPatches.reset(nullptr);

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
    this->mMapSizeX = this->mNumOfGridCellsX * this->mResolution;
    this->mMapSizeY = this->mNumOfGridCellsY * this->mResolution;

    this->mMinPos.mX +=
        (patchIdxMin.mX * this->mPatchSize) * this->mResolution;
    this->mMinPos.mY +=
        (patchIdxMin.mY * this->mPatchSize) * this->mResolution;
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
    assert(this->mNumOfPatchesX >= 0);
    assert(this->mNumOfPatchesY >= 0);

    const int numOfPatches = this->mNumOfPatchesX * this->mNumOfPatchesY;
    assert((numOfPatches > 0 && this->mPatches != nullptr) ||
           (numOfPatches == 0 && this->mPatches == nullptr));

    /* Reset all patches */
    for (int patchIdxY = 0; patchIdxY < this->mNumOfPatchesY; ++patchIdxY)
        for (int patchIdxX = 0; patchIdxX < this->mNumOfPatchesX; ++patchIdxX)
            this->PatchAt(patchIdxX, patchIdxY).Reset();
}

/* Check if the grid cell index is inside the map */
template <typename T>
bool GridMap<T>::IsInside(int idxX, int idxY) const
{
    return (idxX >= 0 && idxX < this->mNumOfGridCellsX) &&
           (idxY >= 0 && idxY < this->mNumOfGridCellsY);
}

/* Convert the grid cell index into the point in world frame
 * The returned point is the bottom-left of the grid cell */
template <typename T>
Point2D<double> GridMap<T>::GridCellIndexToWorldCoordinate(
    int idxX, int idxY) const
{
    assert(this->mResolution > 0.0);

    const double mapX = this->mMinPos.mX + this->mResolution * idxX;
    const double mapY = this->mMinPos.mY + this->mResolution * idxY;

    return Point2D<double> { mapX, mapY };
}

/* Convert the point in world frame to the grid cell index */
template <typename T>
Point2D<int> GridMap<T>::WorldCoordinateToGridCellIndex(
    double mapX, double mapY) const
{
    assert(this->mResolution > 0.0);

    const int cellIdxX = static_cast<int>(std::floor(
        (mapX - this->mMinPos.mX) / this->mResolution));
    const int cellIdxY = static_cast<int>(std::floor(
        (mapY - this->mMinPos.mY) / this->mResolution));
    
    return Point2D<int> { cellIdxX, cellIdxY };
}

/* Convert the point in world frame to the
 * floating-point grid cell index */
template <typename T>
Point2D<double> GridMap<T>::WorldCoordinateToGridCellIndexFloat(
    double mapX, double mapY) const
{
    assert(this->mResolution > 0.0);

    const double cellIdxX = (mapX - this->mMinPos.mX) / this->mResolution;
    const double cellIdxY = (mapY - this->mMinPos.mY) / this->mResolution;

    return Point2D<double> { cellIdxX, cellIdxY };
}

/* Get the grid cell of the specified index */
template <typename T>
typename GridMap<T>::GridCellType& GridMap<T>::GridCellAt(
    int idxX, int idxY)
{
    assert(this->mPatchSize > 0);
    assert(this->IsInside(idxX, idxY));

    const Point2D<int> patchIdx = this->GridCellIndexToPatchIndex(idxX, idxY);
    Patch<T>& patch = this->PatchAt(patchIdx);

    /* Allocate patch if necessary */
    if (!patch.IsAllocated())
        patch.Allocate(this->mPatchSize);

    return patch.At(idxX % this->mPatchSize,
                    idxY % this->mPatchSize);
}

/* Get the grid cell of the specified index */
template <typename T>
const typename GridMap<T>::GridCellType& GridMap<T>::GridCellAt(
    int idxX, int idxY) const
{
    assert(this->mPatchSize > 0);
    assert(this->IsInside(idxX, idxY));

    const Point2D<int> patchIdx = this->GridCellIndexToPatchIndex(idxX, idxY);
    const Patch<T>& patch = this->PatchAt(patchIdx);

    return patch.At(idxX % this->mPatchSize,
                    idxY % this->mPatchSize);
}

/* Get the occupancy probability value of the specified grid cell */
template <typename T>
typename GridMap<T>::ValueType GridMap<T>::Value(
    int idxX, int idxY) const
{
    assert(this->mPatchSize > 0);
    assert(this->IsInside(idxX, idxY));

    const Point2D<int> patchIdx = this->GridCellIndexToPatchIndex(idxX, idxY);
    const Patch<T>& patch = this->PatchAt(patchIdx);

    return patch.Value(idxX % this->mPatchSize,
                       idxY % this->mPatchSize);
}

/* Get the occupancy probability value of the specified grid cell
 * The default value is returned if the specified grid cell is
 * out of bounds or is not yet allocated */
template <typename T>
typename GridMap<T>::ValueType GridMap<T>::Value(
    int idxX, int idxY, ValueType defaultVal) const
{
    assert(this->mPatchSize > 0);

    if (!this->IsInside(idxX, idxY))
        return defaultVal;

    const Point2D<int> patchIdx = this->GridCellIndexToPatchIndex(idxX, idxY);
    const Patch<T>& patch = this->PatchAt(patchIdx);

    return patch.Value(idxX % this->mPatchSize,
                       idxY % this->mPatchSize,
                       defaultVal);
}

/* Update the occupancy probability value of the specified grid cell */
template <typename T>
void GridMap<T>::Update(int idxX, int idxY,
                        const ObservationType& updateVal)
{
    this->GridCellAt(idxX, idxY).Update(updateVal);
}

/* Calculate the distance between two grid cells */
template <typename T>
double GridMap<T>::Distance(int idxX0, int idxY0,
                            int idxX1, int idxY1) const
{
    assert(this->mResolution > 0.0);
    return std::hypot((idxX1 - idxX0) * this->mResolution,
                      (idxY1 - idxY0) * this->mResolution);
}

/* Calculate the squared distance between two grid cells */
template <typename T>
double GridMap<T>::SquaredDistance(int idxX0, int idxY0,
                                   int idxX1, int idxY1) const
{
    assert(this->mResolution > 0.0);
    const double deltaX = (idxX1 - idxX0) * this->mResolution;
    const double deltaY = (idxY1 - idxY0) * this->mResolution;
    return deltaX * deltaX + deltaY * deltaY;
}

/* Convert the grid cell index to the patch index */
template <typename T>
Point2D<int> GridMap<T>::GridCellIndexToPatchIndex(
    int idxX, int idxY) const
{
    assert(this->mPatchSize > 0);
    return Point2D<int> {
        (idxX < 0) ? (idxX / this->mPatchSize - 1) :
                     (idxX / this->mPatchSize),
        (idxY < 0) ? (idxY / this->mPatchSize - 1) :
                     (idxY / this->mPatchSize) };
}

/* Convert the patch index to the grid cell index range */
template <typename T>
void GridMap<T>::PatchIndexToGridCellIndexRange(
    int patchIdxX, int patchIdxY,
    int& minIdxX, int& minIdxY,
    int& maxIdxX, int& maxIdxY) const
{
    assert(this->mPatchSize > 0);
    minIdxX = patchIdxX * this->mPatchSize;
    minIdxY = patchIdxY * this->mPatchSize;
    maxIdxX = minIdxX + this->mPatchSize;
    maxIdxY = minIdxY + this->mPatchSize;
}

/* Check if the patch index is inside the map */
template <typename T>
bool GridMap<T>::PatchIsInside(int patchIdxX, int patchIdxY) const
{
    return (patchIdxX >= 0 && patchIdxX < this->mNumOfPatchesX) &&
           (patchIdxY >= 0 && patchIdxY < this->mNumOfPatchesY);
}

/* Check if the patch is allocated on the heap */
template <typename T>
bool GridMap<T>::PatchIsAllocated(int patchIdxX, int patchIdxY) const
{
    assert(this->PatchIsInside(patchIdxX, patchIdxY));

    const int patchIdx = patchIdxY * this->mNumOfPatchesX + patchIdxX;
    const Patch<T>& patch = this->mPatches[patchIdx];

    return patch.IsAllocated();
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
