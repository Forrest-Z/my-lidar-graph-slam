
/* grid_map_patch.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_GRID_MAP_GRID_MAP_PATCH_HPP
#define MY_LIDAR_GRAPH_SLAM_GRID_MAP_GRID_MAP_PATCH_HPP

#include <cassert>
#include <memory>
#include <utility>

namespace MyLidarGraphSlam {

template <typename T>
class Patch final
{
public:
    /* Type definitions */
    using ValueType = typename T::ValueType;
    using GridCellType = T;

    /* Default constructor */
    Patch() : mSize(-1), mData(nullptr) { }
    /* Destructor */
    ~Patch() = default;

    /* Copy constructor (disabled) */
    Patch(const Patch& other) = delete;
    /* Copy assignment operator (disabled) */
    Patch& operator=(const Patch& other) = delete;

    /* Move constructor */
    Patch(Patch&& other) noexcept;
    /* Move assignment operator */
    Patch& operator=(Patch&& other) noexcept;

    /* Allocate cells */
    void Allocate(int patchSize);

    /* Check if patch is allocated */
    inline bool IsAllocated() const { return this->mData != nullptr; }

    /* Get the size of the patch */
    inline int Size() const { return this->mSize; }
    /* Get the constant pointer to the cell */
    inline const GridCellType* Data() const { return this->mData.get(); }
    /* Get the pointer to the cell */
    inline GridCellType* Data() { return this->mData.get(); }

    /* Get the cell of the patch */
    GridCellType& At(int x, int y);
    const GridCellType& At(int x, int y) const;

    /* Get the cell value of the patch */
    const ValueType Value(int x, int y) const;

    /* Get the cell value of the patch
     * Return the default value if not allocated */
    const ValueType Value(int x, int y,
        const ValueType defaultValue = GridCellType::Unknown) const;
    
    /* Reset all occupancy grid cells */
    void Reset();

private:
    /* Validate the patch */
    inline bool IsValid() const;
    /* Check if the index is inside the patch */
    inline bool IsInside(int idxX, int idxY) const;

private:
    /* Patch size */
    int mSize;

    /* Patch data */
    std::unique_ptr<T[]> mData;
};

/* Move constructor */
template <typename T>
Patch<T>::Patch(Patch<T>&& other) noexcept :
    mSize(other.mSize),
    mData(std::move(other.mData))
{
}

/* Move assignment operator */
template <typename T>
Patch<T>& Patch<T>::operator=(Patch<T>&& other) noexcept
{
    if (this == &other)
        return *this;
    
    this->mSize = other.mSize;
    this->mData = std::move(other.mData);

    return *this;
}

/* Allocate patch data */
template <typename T>
void Patch<T>::Allocate(int patchSize)
{    
    assert(patchSize > 0);

    this->mSize = patchSize;
    this->mData.reset(new T[patchSize * patchSize]);

    assert(this->mData != nullptr);
}

/* Get the cell value in patch */
template <typename T>
typename Patch<T>::GridCellType& Patch<T>::At(int x, int y)
{
    assert(this->IsValid());
    assert(this->IsInside(x, y));

    return this->mData[y * this->mSize + x];
}

/* Get the cell value in patch */
template <typename T>
const typename Patch<T>::GridCellType& Patch<T>::At(int x, int y) const
{
    assert(this->IsValid());
    assert(this->IsInside(x, y));

    return this->mData[y * this->mSize + x];
}

/* Get the cell value of the patch */
template <typename T>
const typename Patch<T>::ValueType Patch<T>::Value(int x, int y) const
{
    assert(this->IsValid());
    assert(this->IsInside(x, y));
    
    return this->mData[y * this->mSize + x].Value();
}

/* Get the cell value of the patch
 * Return the default value if not allocated */
template <typename T>
const typename Patch<T>::ValueType Patch<T>::Value(
    int x, int y, const ValueType defaultValue) const
{
    /* Return the default occupancy value if not allocated */
    if (!this->IsValid())
        return defaultValue;
    
    assert(this->IsInside(x, y));
    return this->mData[y * this->mSize + x].Value();
}

/* Reset all occupancy grid cells */
template <typename T>
void Patch<T>::Reset()
{
    /* Do not reset if not allocated */
    if (!this->IsValid())
        return;

    /* Reset all grid cells */
    for (int idxY = 0; idxY < this->mSize; ++idxY)
        for (int idxX = 0; idxX < this->mSize; ++idxX)
            this->At(idxX, idxY).Reset();
}

/* Validate the patch */
template <typename T>
bool Patch<T>::IsValid() const
{
    return (this->mSize > 0) && (this->mData != nullptr);
}

/* Check if the index is inside the patch */
template <typename T>
bool Patch<T>::IsInside(int idxX, int idxY) const
{
    return (idxX >= 0 && idxX < this->mSize) &&
           (idxY >= 0 && idxY < this->mSize);
}

} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_GRID_MAP_GRID_MAP_PATCH_HPP */
