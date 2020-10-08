
/* pose_graph_id.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_POSE_GRAPH_ID_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_POSE_GRAPH_ID_HPP

namespace MyLidarGraphSlam {
namespace Mapping {

/*
 * LocalMapId struct represents an Id value for a local grid map
 */
struct LocalMapId final
{
    /* Constructor */
    explicit LocalMapId(const int localMapId) : mId(localMapId) { }

    /* Equality operator */
    inline bool operator==(const LocalMapId& other) const
    { return this->mId == other.mId; }
    /* Inequality operator */
    inline bool operator!=(const LocalMapId& other) const
    { return !operator==(other); }

    /* Less-than comparison operator (for std::map) */
    inline bool operator<(const LocalMapId& other) const
    { return this->mId < other.mId; }
    /* Greater-than comparison operator */
    inline bool operator>(const LocalMapId& other) const
    { return this->mId > other.mId; }
    /* Less-than or equal to comparison operator */
    inline bool operator<=(const LocalMapId& other) const
    { return this->mId <= other.mId; }
    /* Greater-than or equal to comparison operator */
    inline bool operator>=(const LocalMapId& other) const
    { return this->mId >= other.mId; }

    /* Invalid local map Id */
    static constexpr const int Invalid = -1;

    /* Local grid map Id */
    int mId;
};

/*
 * NodeId struct represents an Id value for a pose graph node
 */
struct NodeId final
{
    /* Constructor */
    explicit NodeId(const int nodeId) : mId(nodeId) { }

    /* Equality operator */
    inline bool operator==(const NodeId& other) const
    { return this->mId == other.mId; }
    /* Inequality operator */
    inline bool operator!=(const NodeId& other) const
    { return !operator==(other); }

    /* Less-than comparison operator (for std::map) */
    inline bool operator<(const NodeId& other) const
    { return this->mId < other.mId; }
    /* Greater-than comparison operator */
    inline bool operator>(const NodeId& other) const
    { return this->mId > other.mId; }
    /* Less-than or equal to comparison operator */
    inline bool operator<=(const NodeId& other) const
    { return this->mId <= other.mId; }
    /* Greater-than or equal to comparison operator */
    inline bool operator>=(const NodeId& other) const
    { return this->mId >= other.mId; }

    /* Invalid node Id */
    static constexpr const int Invalid = -1;

    /* Pose graph node Id */
    int mId;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_POSE_GRAPH_ID_HPP */
