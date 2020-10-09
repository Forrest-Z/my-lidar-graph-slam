
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
    GridMap(const double mapResolution,
            const int patchSize,
            const int initialNumOfCellsX,
            const int initialNumOfCellsY,
            const Point2D<double>& localCenterPos);

    /* Constructor with map size */
    GridMap(const double mapResolution,
            const int patchSize,
            const double mapSizeX,
            const double mapSizeY,
            const Point2D<double>& localCenterPos);

    /* Constructor with positions */
    GridMap(const double mapResolution,
            const int patchSize,
            const double localMinX,
            const double localMinY,
            const double localMaxX,
            const double localMaxY);

    /* Constructor with internal parameters */
    GridMap(const double mapResolution,
            const int patchSize,
            const int numOfPatchesX,
            const int numOfPatchesY,
            const double localMinX,
            const double localMinY);

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
    void Resize(const double localMinX,
                const double localMinY,
                const double localMaxX,
                const double localMaxY);
    /* Expand the map if necessary */
    void Expand(const double localMinX,
                const double localMinY,
                const double localMaxX,
                const double localMaxY,
                const double enlargeStep = 5.0);

    /* Reset occupancy probability values of all grid cells */
    void Reset();

    /* Get the unknown occupancy probability value */
    ValueType UnknownValue() const override { return GridCellType::Unknown; }

    /* Check if the grid cell index is inside the map */
    bool IsInside(int idxX, int idxY) const override;
    /* Check if the grid cell index is inside the map */
    bool IsInside(const Point2D<int>& gridCellIdx) const override
    { return this->IsInside(gridCellIdx.mX, gridCellIdx.mY); }

    /* Check if the point in the local frame is inside the map */
    bool IsInside(double localMapX, double localMapY) const override
    { return this->IsInside(
        this->LocalPosToGridCellIndex(localMapX, localMapY)); }
    /* Check if the point in the local frame is inside the map */
    bool IsInside(const Point2D<double>& localMapPos) const override
    { return this->IsInside(localMapPos.mX, localMapPos.mY); }

    /* Check if the grid cell is allocated on the heap */
    bool IsAllocated(int idxX, int idxY) const override
    { return this->PatchIsAllocated(
        this->GridCellIndexToPatchIndex(idxX, idxY)); }
    /* Check if the grid cell is allocated on the heap */
    bool IsAllocated(const Point2D<int>& gridCellIdx) const override
    { return this->IsAllocated(gridCellIdx.mX, gridCellIdx.mY); }

    /* Convert the grid cell index into the point in the local frame
     * The returned point is the minimum position of the grid cell */
    Point2D<double> GridCellIndexToLocalPos(
        int idxX, int idxY) const override;
    /* Convert the grid cell index into the point in the local frame
     * The returned point is the minimum position of the grid cell */
    Point2D<double> GridCellIndexToLocalPos(
        const Point2D<int>& gridCellIdx) const override
    { return this->GridCellIndexToLocalPos(
        gridCellIdx.mX, gridCellIdx.mY); }

    /* Convert the point in the local frame to the grid cell index */
    Point2D<int> LocalPosToGridCellIndex(
        double localMapX, double localMapY) const override;
    /* Convert the point in the local frame to the grid cell index */
    Point2D<int> LocalPosToGridCellIndex(
        const Point2D<double>& localMapPos) const override
    { return this->LocalPosToGridCellIndex(
        localMapPos.mX, localMapPos.mY); }

    /* Convert the point in the local frame to the
     * floating-point grid cell index */
    Point2D<double> LocalPosToGridCellIndexFloat(
        double localMapX, double localMapY) const override;
    /* Convert the point in the local frame to the
     * floating-point grid cell index */
    Point2D<double> LocalPosToGridCellIndexFloat(
        const Point2D<double>& localMapPos) const override
    { return this->LocalPosToGridCellIndexFloat(
        localMapPos.mX, localMapPos.mY); }

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
    void Update(double localMapX, double localMapY,
                const ObservationType& updateVal)
    { this->Update(this->LocalPosToGridCellIndex(localMapX, localMapY),
                   updateVal); }
    /* Update the occupancy probability value of the specified point */
    void Update(const Point2D<double>& localMapPos,
                const ObservationType& updateVal)
    { this->Update(localMapPos.mX, localMapPos.mY, updateVal); }

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
    { return this->GridCellIndexToPatchIndex(
        gridCellIdx.mX, gridCellIdx.mY); }

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

    /* Compute the actual grid map size */
    void ComputeActualMapSize(Point2D<int>& patchIdxMin,
                              Point2D<int>& patchIdxMax,
                              Point2D<int>& gridCellIdxMin,
                              Point2D<int>& gridCellIdxMax,
                              Point2D<int>& mapSizeInPatches,
                              Point2D<int>& mapSizeInGridCells,
                              Point2D<double>& mapSizeInMeters) const;

    /* Compute the bounding box in a world coordinate frame */
    void ComputeBoundingBox(const RobotPose2D<double>& globalPose,
                            Point2D<double>& globalMinPos,
                            Point2D<double>& globalMaxPos) const;

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

    /* Get the minimum position of the map in the local coordinate */
    inline const Point2D<double>& LocalMinPos() const override
    { return this->mLocalMinPos; }

    /* Get the maximum position of the map in the local coordinate */
    inline const Point2D<double> LocalMaxPos() const
    { return this->GridCellIndexToLocalPos(
        this->mNumOfGridCellsX, this->mNumOfGridCellsY); }

private:
    /* Get the active area of this grid map */
    void ComputeActiveArea(Point2D<int>& patchIdxMin,
                           Point2D<int>& patchIdxMax,
                           Point2D<int>& gridCellIdxMin,
                           Point2D<int>& gridCellIdxMax) const;

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
    /* Minimum position of the map in the local frame
     * corresponding to the origin grid cell (0, 0)
     * No relationship to pose graph nodes */
    Point2D<double>             mLocalMinPos;
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
    mLocalMinPos(0.0, 0.0),
    mPatches(nullptr)
{
}

/* Constructor with number of grid cells */
template <typename T>
GridMap<T>::GridMap(const double mapResolution,
                    const int patchSize,
                    const int initialNumOfCellsX,
                    const int initialNumOfCellsY,
                    const Point2D<double>& localCenterPos)
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
    const int numOfCellsX = std::max(0, initialNumOfCellsX);
    const int numOfCellsY = std::max(0, initialNumOfCellsY);

    this->mNumOfPatchesX = static_cast<int>(std::ceil(
        static_cast<double>(numOfCellsX) /
        static_cast<double>(patchSize)));
    this->mNumOfPatchesY = static_cast<int>(std::ceil(
        static_cast<double>(numOfCellsY) /
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
    this->mLocalMinPos.mX = localCenterPos.mX - idxOffsetX * this->mResolution;
    this->mLocalMinPos.mY = localCenterPos.mY - idxOffsetY * this->mResolution;

    /* Allocate the memory for map patches if necessary */
    const int numOfPatches = this->mNumOfPatchesX * this->mNumOfPatchesY;

    if (numOfPatches > 0)
        this->mPatches.reset(new Patch<T>[numOfPatches]);
    else
        this->mPatches.reset(nullptr);
}

/* Constructor with map size */
template <typename T>
GridMap<T>::GridMap(const double mapResolution,
                    const int patchSize,
                    const double mapSizeX,
                    const double mapSizeY,
                    const Point2D<double>& localCenterPos)
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
    this->mLocalMinPos.mX = localCenterPos.mX - idxOffsetX * this->mResolution;
    this->mLocalMinPos.mY = localCenterPos.mY - idxOffsetY * this->mResolution;

    /* Allocate the memory for map patches (not grid cells) */
    const int numOfPatches = this->mNumOfPatchesX * this->mNumOfPatchesY;

    if (numOfPatches > 0)
        this->mPatches.reset(new Patch<T>[numOfPatches]);
    else
        this->mPatches.reset(nullptr);
}

/* Constructor with positions */
template <typename T>
GridMap<T>::GridMap(const double mapResolution,
                    const int patchSize,
                    const double localMinX,
                    const double localMinY,
                    const double localMaxX,
                    const double localMaxY)
{
    /* Input validity checks */
    assert(mapResolution > 0.0);
    assert(patchSize > 0);
    assert(localMinX <= localMaxX);
    assert(localMinY <= localMaxY);

    /* Set the map resolution (size of the each grid cell) */
    this->mResolution = mapResolution;
    /* Set the patch size (in number of grid cells) */
    this->mPatchSize = patchSize;

    /* Empty grid map with no patch is allowed */
    const double mapSizeX = std::max(0.0, localMaxX - localMinX);
    const double mapSizeY = std::max(0.0, localMaxY - localMinY);

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
    this->mLocalMinPos.mX = localMinX;
    this->mLocalMinPos.mY = localMinY;

    /* Allocate the memory for map patches */
    const int numOfPatches = this->mNumOfPatchesX * this->mNumOfPatchesY;

    if (numOfPatches > 0)
        this->mPatches.reset(new Patch<T>[numOfPatches]);
    else
        this->mPatches.reset(nullptr);
}

/* Constructor with internal parameters */
template <typename T>
GridMap<T>::GridMap(const double mapResolution,
                    const int patchSize,
                    const int numOfPatchesX,
                    const int numOfPatchesY,
                    const double localMinX,
                    const double localMinY)
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
    this->mLocalMinPos.mX = localMinX;
    this->mLocalMinPos.mY = localMinY;

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
    GridMapBase<typename T::ValueType>(other),
    mResolution(0.0),
    mPatchSize(0),
    mNumOfPatchesX(0),
    mNumOfPatchesY(0),
    mNumOfGridCellsX(0),
    mNumOfGridCellsY(0),
    mMapSizeX(0.0),
    mMapSizeY(0.0),
    mLocalMinPos(0.0, 0.0),
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
    this->mLocalMinPos = other.mLocalMinPos;

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
    mLocalMinPos(other.mLocalMinPos),
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
    this->mLocalMinPos = other.mLocalMinPos;
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
                        gridMap.LocalMinPos().mX, gridMap.LocalMinPos().mY };
}

/* Resize the map */
template <typename T>
void GridMap<T>::Resize(const double localMinX,
                        const double localMinY,
                        const double localMaxX,
                        const double localMaxY)
{
    assert(localMinX <= localMaxX);
    assert(localMinY <= localMaxY);

    /* Calculate the grid cell index (can be negative) */
    const Point2D<int> gridCellIdxMin =
        this->LocalPosToGridCellIndex(localMinX, localMinY);
    const Point2D<int> gridCellIdxMax =
        this->LocalPosToGridCellIndex(localMaxX, localMaxY);

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

    this->mLocalMinPos.mX +=
        (patchIdxMin.mX * this->mPatchSize) * this->mResolution;
    this->mLocalMinPos.mY +=
        (patchIdxMin.mY * this->mPatchSize) * this->mResolution;
}

/* Expand the map if necessary */
template <typename T>
void GridMap<T>::Expand(const double localMinX,
                        const double localMinY,
                        const double localMaxX,
                        const double localMaxY,
                        const double enlargeStep)
{
    assert(localMinX <= localMaxX);
    assert(localMinY <= localMaxY);

    if (this->IsInside(localMinX, localMinY) &&
        this->IsInside(localMaxX, localMaxY))
        return;

    Point2D<double> minPos = this->GridCellIndexToLocalPos(0, 0);
    Point2D<double> maxPos = this->GridCellIndexToLocalPos(
        this->mNumOfGridCellsX, this->mNumOfGridCellsY);

    minPos.mX = (localMinX < minPos.mX) ? localMinX - enlargeStep : minPos.mX;
    minPos.mY = (localMinY < minPos.mY) ? localMinY - enlargeStep : minPos.mY;
    maxPos.mX = (localMaxX > maxPos.mX) ? localMaxX + enlargeStep : maxPos.mX;
    maxPos.mY = (localMaxY > maxPos.mY) ? localMaxY + enlargeStep : maxPos.mY;

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

/* Convert the grid cell index into the point in the local frame
 * The returned point is the minimum position of the grid cell */
template <typename T>
Point2D<double> GridMap<T>::GridCellIndexToLocalPos(
    int idxX, int idxY) const
{
    assert(this->mResolution > 0.0);

    const double localMapX = this->mLocalMinPos.mX + this->mResolution * idxX;
    const double localMapY = this->mLocalMinPos.mY + this->mResolution * idxY;

    return Point2D<double> { localMapX, localMapY };
}

/* Convert the point in the local frame to the grid cell index */
template <typename T>
Point2D<int> GridMap<T>::LocalPosToGridCellIndex(
    double localMapX, double localMapY) const
{
    assert(this->mResolution > 0.0);

    const int cellIdxX = static_cast<int>(std::floor(
        (localMapX - this->mLocalMinPos.mX) / this->mResolution));
    const int cellIdxY = static_cast<int>(std::floor(
        (localMapY - this->mLocalMinPos.mY) / this->mResolution));

    return Point2D<int> { cellIdxX, cellIdxY };
}

/* Convert the point in the local frame to the
 * floating-point grid cell index */
template <typename T>
Point2D<double> GridMap<T>::LocalPosToGridCellIndexFloat(
    double localMapX, double localMapY) const
{
    assert(this->mResolution > 0.0);

    const double cellIdxX =
        (localMapX - this->mLocalMinPos.mX) / this->mResolution;
    const double cellIdxY =
        (localMapY - this->mLocalMinPos.mY) / this->mResolution;

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

/* Compute the actual grid map size */
template <typename T>
void GridMap<T>::ComputeActualMapSize(
    Point2D<int>& patchIdxMin,
    Point2D<int>& patchIdxMax,
    Point2D<int>& gridCellIdxMin,
    Point2D<int>& gridCellIdxMax,
    Point2D<int>& mapSizeInPatches,
    Point2D<int>& mapSizeInGridCells,
    Point2D<double>& mapSizeInMeters) const
{
    /* Compute the active area of this grid map */
    this->ComputeActiveArea(patchIdxMin, patchIdxMax,
                            gridCellIdxMin, gridCellIdxMax);

    /* Compute the actual map size */
    mapSizeInPatches.mX = patchIdxMax.mX - patchIdxMin.mX;
    mapSizeInPatches.mY = patchIdxMax.mY - patchIdxMin.mY;
    mapSizeInGridCells.mX = gridCellIdxMax.mX - gridCellIdxMin.mX;
    mapSizeInGridCells.mY = gridCellIdxMax.mY - gridCellIdxMin.mY;

    assert(mapSizeInGridCells.mX == mapSizeInPatches.mX * this->mPatchSize);
    assert(mapSizeInGridCells.mY == mapSizeInPatches.mY * this->mPatchSize);

    mapSizeInMeters.mX = mapSizeInGridCells.mX * this->mResolution;
    mapSizeInMeters.mY = mapSizeInGridCells.mY * this->mResolution;
}

/* Compute the bounding box in a world coordinate frame */
template <typename T>
void GridMap<T>::ComputeBoundingBox(
    const RobotPose2D<double>& globalPose,
    Point2D<double>& globalMinPos,
    Point2D<double>& globalMaxPos) const
{
    const Point2D<double> rotatedMinPos =
        Rotate(this->LocalMinPos(), globalPose.mTheta);
    const Point2D<double> rotatedMaxPos =
        Rotate(this->LocalMaxPos(), globalPose.mTheta);

    const Point2D<double> cornerPos0 {
        globalPose.mX + rotatedMinPos.mX,
        globalPose.mY + rotatedMinPos.mY };
    const Point2D<double> cornerPos1 {
        globalPose.mX + rotatedMaxPos.mX,
        globalPose.mY + rotatedMaxPos.mY };

    globalMinPos.mX = std::min(cornerPos0.mX, cornerPos1.mX);
    globalMinPos.mY = std::min(cornerPos0.mY, cornerPos1.mY);
    globalMaxPos.mX = std::max(cornerPos0.mX, cornerPos1.mX);
    globalMaxPos.mY = std::max(cornerPos0.mY, cornerPos1.mY);
}

/* Get the active area of this grid map */
template <typename T>
void GridMap<T>::ComputeActiveArea(
    Point2D<int>& patchIdxMin,
    Point2D<int>& patchIdxMax,
    Point2D<int>& gridCellIdxMin,
    Point2D<int>& gridCellIdxMax) const
{
    /* Get the range of the patch index (bounding box)
     * [patchIdxMin.mX, patchIdxMax.mX], [patchIdxMin.mY, patchIdxMax.mY] */
    patchIdxMin.mX = std::numeric_limits<int>::max();
    patchIdxMin.mY = std::numeric_limits<int>::max();
    patchIdxMax.mX = std::numeric_limits<int>::min();
    patchIdxMax.mY = std::numeric_limits<int>::min();

    for (int y = 0; y < this->mNumOfPatchesY; ++y) {
        for (int x = 0; x < this->mNumOfPatchesX; ++x) {
            if (!this->PatchIsAllocated(x, y))
                continue;

            patchIdxMin.mX = std::min(patchIdxMin.mX, x);
            patchIdxMin.mY = std::min(patchIdxMin.mY, y);
            patchIdxMax.mX = std::max(patchIdxMax.mX, x);
            patchIdxMax.mY = std::max(patchIdxMax.mY, y);
        }
    }

    /* Convert the patch index ranges to grid cell index ranges */
    Point2D<int> gridCellIdxTmp;
    this->PatchIndexToGridCellIndexRange(
        patchIdxMin, gridCellIdxMin, gridCellIdxTmp);
    this->PatchIndexToGridCellIndexRange(
        patchIdxMax, gridCellIdxTmp, gridCellIdxMax);

    /* Correct the maximum grid patch index */
    patchIdxMax.mX += 1;
    patchIdxMax.mY += 1;
}

} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_GRID_MAP_GRID_MAP_HPP */
