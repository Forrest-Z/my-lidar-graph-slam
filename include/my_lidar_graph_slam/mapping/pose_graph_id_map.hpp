
/* pose_graph_id_map.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_POSE_GRAPH_ID_MAP_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_POSE_GRAPH_ID_MAP_HPP

#include <cstdint>
#include <iterator>
#include <map>
#include <memory>

#include "my_lidar_graph_slam/util.hpp"

namespace MyLidarGraphSlam {
namespace Mapping {

/*
 * Range class holds a pair of iterators (begin() and end())
 */
template <typename IteratorType>
class Range
{
public:
    /* Constructor */
    Range(const IteratorType& beginIt, const IteratorType& endIt) :
        mBeginIt(beginIt), mEndIt(endIt) { }

    /* Copy constructor */
    Range(const Range&) = default;
    /* Copy assignment operator */
    Range& operator=(const Range&) = default;
    /* Move constructor */
    Range(Range&&) = default;
    /* Move assignment operator */
    Range& operator=(Range&&) = default;

    /* Get the iterator to the begin element */
    inline IteratorType& begin() noexcept
    { return this->mBeginIt; }
    /* Get the iterator to the begin element */
    inline const IteratorType& begin() const noexcept
    { return this->mBeginIt; }

    /* Get the iterator to the end element */
    inline IteratorType& end() noexcept
    { return this->mEndIt; }
    /* Get the iterator to the end element */
    inline const IteratorType& end() const noexcept
    { return this->mEndIt; }

private:
    /* Iterator to the begin element */
    IteratorType mBeginIt;
    /* Iterator to the end element */
    IteratorType mEndIt;
};

/*
 * IdMap class manages the mappings between the pose graph node and
 * its associated data, which tries hard to look like STL-compatible container
 * NodeId or LocalMapId is allowed for IdType
 */
template <typename IdType, typename DataType>
class IdMap final
{
public:
    /* Type declarations */
    using MapType = std::map<IdType, DataType>;

    struct IdDataPair
    {
        /* Constructor */
        IdDataPair(const IdType nodeId, DataType& data) :
            mId(nodeId), mData(data) { }

        const IdType mId;
        DataType&    mData;
    };

    struct ConstIdDataPair
    {
        /* Constructor */
        ConstIdDataPair(const IdType nodeId, const DataType& data) :
            mId(nodeId), mData(data) { }

        const IdType    mId;
        const DataType& mData;
    };

    class Iterator
    {
    public:
        /* Type declarations */
        using value_type = IdDataPair;
        using reference = IdDataPair&;
        using pointer = std::unique_ptr<IdDataPair>;
        using difference_type = std::ptrdiff_t;
        using iterator_category = std::bidirectional_iterator_tag;

        /* Constructor with the std::map iterator */
        Iterator(const typename MapType::iterator& mapIt) :
            mMapIt(mapIt) { }

        /* Dereference operator */
        inline IdDataPair operator*() const
        { return IdDataPair(this->mMapIt->first, this->mMapIt->second); }
        /* Member access operator */
        inline std::unique_ptr<IdDataPair> operator->() const
        { return std::make_unique<IdDataPair>(this->operator*()); }

        /* Equality operator */
        inline bool operator==(const Iterator& other) const
        { return this->mMapIt == other.mMapIt; }
        /* Inequality operator */
        inline bool operator!=(const Iterator& other) const
        { return !operator==(other); }

        /* Increment operator */
        inline Iterator operator++(int)
        { return Iterator(this->mMapIt++); }
        /* Decrement operator */
        inline Iterator operator--(int)
        { return Iterator(this->mMapIt--); }

        /* Increment operator */
        inline Iterator& operator++()
        { ++this->mMapIt; return *this; }
        /* Decrement operator */
        inline Iterator& operator--()
        { --this->mMapIt; return *this; }

    private:
        typename MapType::iterator mMapIt;
    };

    class ConstIterator
    {
    public:
        /* Type declarations */
        using value_type = ConstIdDataPair;
        using reference = ConstIdDataPair&;
        using pointer = std::unique_ptr<const ConstIdDataPair>;
        using difference_type = std::ptrdiff_t;
        using iterator_category = std::bidirectional_iterator_tag;

        /* Constructor with the std::map const iterator */
        ConstIterator(const typename MapType::const_iterator& mapIt) :
            mMapIt(mapIt) { }

        /* Dereference operator */
        inline ConstIdDataPair operator*() const
        { return ConstIdDataPair(this->mMapIt->first, this->mMapIt->second); }
        /* Member access operator */
        inline std::unique_ptr<const ConstIdDataPair> operator->() const
        { return std::make_unique<const ConstIdDataPair>(this->operator*()); }

        /* Equality operator */
        inline bool operator==(const ConstIterator& other) const
        { return this->mMapIt == other.mMapIt; }
        /* Inequality operator */
        inline bool operator!=(const ConstIterator& other) const
        { return !operator==(other); }

        /* Increment operator */
        inline ConstIterator operator++(int)
        { return ConstIterator(this->mMapIt++); }
        /* Decrement operator */
        inline ConstIterator operator--(int)
        { return ConstIterator(this->mMapIt--); }

        /* Increment operator */
        inline ConstIterator& operator++()
        { ++this->mMapIt; return *this; }
        /* Decrement operator */
        inline ConstIterator& operator--()
        { --this->mMapIt; return *this; }

    private:
        typename MapType::const_iterator mMapIt;
    };

    /* Reverse iterators */
    using ReverseIterator = std::reverse_iterator<Iterator>;
    using ConstReverseIterator = std::reverse_iterator<ConstIterator>;

    /* Constructor */
    IdMap() = default;
    /* Destructor */
    ~IdMap() = default;

    /* Get an iterator to the first element */
    inline Iterator begin() noexcept
    { return Iterator(this->mMap.begin()); }
    /* Get a const iterator to the first element */
    inline ConstIterator begin() const noexcept
    { return ConstIterator(this->mMap.begin()); }
    /* Get a const iterator to the first element */
    inline ConstIterator cbegin() const noexcept
    { return ConstIterator(this->mMap.cbegin()); }

    /* Get an iterator to the end element */
    inline Iterator end() noexcept
    { return Iterator(this->mMap.end()); }
    /* Get a const iterator to the end element */
    inline ConstIterator end() const noexcept
    { return ConstIterator(this->mMap.end()); }
    /* Get a const iterator to the end element */
    inline ConstIterator cend() const noexcept
    { return ConstIterator(this->mMap.cend()); }

    /* Get an iterator to the first element of the reversed map */
    inline ReverseIterator rbegin() noexcept
    { return ReverseIterator(this->end()); }
    /* Get a const iterator to the first element of the reversed map */
    inline ConstReverseIterator rbegin() const noexcept
    { return ConstReverseIterator(this->end()); }
    /* Get a const iterator to the first element of the reversed map */
    inline ConstReverseIterator crbegin() const noexcept
    { return ConstReverseIterator(this->cend()); }

    /* Get an iterator to the end element of the reversed map */
    inline ReverseIterator rend() noexcept
    { return ReverseIterator(this->begin()); }
    /* Get a const iterator to the end element of the reversed map */
    inline ConstReverseIterator rend() const noexcept
    { return ConstReverseIterator(this->begin()); }
    /* Get a const iterator to the end element of the reversed map */
    inline ConstReverseIterator crend() const noexcept
    { return ConstReverseIterator(this->cbegin()); }

    /* Check if the map is empty */
    inline bool empty() const { return this->mMap.empty(); }
    /* Get the number of the elements in this map */
    inline std::size_t size() const { return this->mMap.size(); }

    /* Clear the map */
    inline void clear() { this->mMap.clear(); }

    /* Get the element with the specified Id */
    inline DataType& at(const IdType& id)
    { return this->mMap.at(id); }
    /* Get the element with the specified Id */
    inline const DataType& at(const IdType& id) const
    { return this->mMap.at(id); }

    /* Get the number of the elements with the specified Id (0 or 1) */
    inline std::size_t count(const IdType& id) const
    { return this->mMap.count(id); }

    /* Get an iterator to the element with the specified Id if exists */
    inline Iterator find(const IdType& id)
    { return Iterator(this->mMap.find(id)); }
    /* Get a const iterator to the element with the specified Id if exists */
    inline ConstIterator find(const IdType& id) const
    { return ConstIterator(this->mMap.find(id)); }

    /* Check if the element with the specified Id exists */
    inline bool contains(const IdType& id) const
    { return this->count(id) > 0; }

    /* Get an iterator to the first element not less than the given Id */
    inline Iterator lower_bound(const IdType& id)
    { return Iterator(this->mMap.lower_bound(id)); }
    /* Get an iterator to the first element not less than the given Id */
    inline ConstIterator lower_bound(const IdType& id) const
    { return ConstIterator(this->mMap.lower_bound(id)); }

    /* Get an iterator to the first element greater than the given Id */
    inline Iterator upper_bound(const IdType& id)
    { return Iterator(this->mMap.upper_bound(id)); }
    /* Get an iterator to the first element greater than the given Id */
    inline ConstIterator upper_bound(const IdType& id) const
    { return ConstIterator(this->mMap.upper_bound(id)); }

    /* Append a new element to this map */
    void Append(const IdType& id, const DataType& data);
    /* Append a new element to this map */
    template <typename... Args>
    void Append(const IdType& id, Args&&... args);

    /* Get the iterator pointing the specified Id */
    Iterator IteratorAt(const IdType& id);
    /* Get the iterator pointing the specified Id */
    ConstIterator IteratorAt(const IdType& id) const;
    /* Get the reverse iterator pointing the specified Id */
    ReverseIterator ReverseIteratorAt(const IdType& id)
    { return ReverseIterator(this->IteratorAt(id)); }
    /* Get the reverse iterator pointing the specified Id */
    ConstReverseIterator ReverseIteratorAt(const IdType& id) const
    { return ConstReverseIterator(this->IteratorAt(id)); }

    /* Create a range from a pair of Ids */
    Range<Iterator> RangeFromId(const IdType& beginId,
                                const IdType& endId)
    { return Range<Iterator>(this->IteratorAt(beginId),
                             this->IteratorAt(endId)); }
    /* Create a range [beginId, endId) from a pair of Ids */
    Range<ConstIterator> RangeFromId(const IdType& beginId,
                                     const IdType& endId) const
    { return Range<ConstIterator>(this->IteratorAt(beginId),
                                  this->IteratorAt(endId)); }

    /* Get the minimum Id in this map */
    IdType IdMin() const;
    /* Get the maximum Id in this map */
    IdType IdMax() const;

    /* Get the first element with the minimum Id */
    DataType& Front();
    /* Get the first element with the minimum Id */
    const DataType& Front() const;
    /* Get the last element with the maximum Id */
    DataType& Back();
    /* Get the last element with the maximum Id */
    const DataType& Back() const;

private:
    /* Actual map container */
    MapType mMap;
};

/* Append a new element to this map */
template <typename IdType, typename DataType>
void IdMap<IdType, DataType>::Append(const IdType& id, const DataType& data)
{
    Assert(id.mId != IdType::Invalid);
    auto emplaceResult = this->mMap.emplace(id, data);
    Assert(emplaceResult.second);
}

/* Append a new element to this map */
template <typename IdType, typename DataType>
template <typename... Args>
void IdMap<IdType, DataType>::Append(const IdType& id, Args&&... args)
{
    Assert(id.mId != IdType::Invalid);
    auto emplaceResult = this->mMap.emplace(
        std::piecewise_construct,
        std::forward_as_tuple(id),
        std::forward_as_tuple(id, std::forward<Args>(args)...));
    Assert(emplaceResult.second);
}

/* Get the iterator pointing the specified Id */
template <typename IdType, typename DataType>
typename IdMap<IdType, DataType>::Iterator
    IdMap<IdType, DataType>::IteratorAt(const IdType& id)
{
    auto elementIt = this->find(id);
    Assert(elementIt != this->end());
    return elementIt;
}

/* Get the iterator pointing the specified Id */
template <typename IdType, typename DataType>
typename IdMap<IdType, DataType>::ConstIterator
    IdMap<IdType, DataType>::IteratorAt(const IdType& id) const
{
    auto elementIt = this->find(id);
    Assert(elementIt != this->end());
    return elementIt;
}

/* Get the minimum Id in this map */
template <typename IdType, typename DataType>
IdType IdMap<IdType, DataType>::IdMin() const
{
    /* Make sure that the map is not empty */
    Assert(!this->empty());
    /* Return the minimum Id */
    return this->begin()->mId;
}

/* Get the maximum Id in this map */
template <typename IdType, typename DataType>
IdType IdMap<IdType, DataType>::IdMax() const
{
    /* Make sure that the map is not empty */
    Assert(!this->empty());
    /* Return the maximum Id */
    return this->rbegin()->mId;
}

/* Get the first element with the minimum Id */
template <typename IdType, typename DataType>
DataType& IdMap<IdType, DataType>::Front()
{
    /* Make sure that the map is not empty */
    Assert(!this->empty());
    /* Return the first element */
    return this->begin()->mData;
}

/* Get the first element with the minimum Id */
template <typename IdType, typename DataType>
const DataType& IdMap<IdType, DataType>::Front() const
{
    /* Make sure that the map is not empty */
    Assert(!this->empty());
    /* Return the first element */
    return this->begin()->mData;
}

/* Get the last element with the largest Id */
template <typename IdType, typename DataType>
DataType& IdMap<IdType, DataType>::Back()
{
    /* Make sure that the map is not empty */
    Assert(!this->empty());
    /* Return the last element */
    return this->rbegin()->mData;
}

/* Get the last element with the largest Id */
template <typename IdType, typename DataType>
const DataType& IdMap<IdType, DataType>::Back() const
{
    /* Make sure that the map is not empty */
    Assert(!this->empty());
    /* Return the last element */
    return this->rbegin()->mData;
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_POSE_GRAPH_ID_MAP_HPP */
