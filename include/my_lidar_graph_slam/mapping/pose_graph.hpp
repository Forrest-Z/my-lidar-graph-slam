
/* pose_graph.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_POSE_GRAPH_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_POSE_GRAPH_HPP

#include "my_lidar_graph_slam/pose.hpp"
#include "my_lidar_graph_slam/sensor/sensor_data.hpp"

#include <memory>
#include <vector>

#include <Eigen/Core>

namespace MyLidarGraphSlam {
namespace Mapping {

/*
 * NodePosition struct represents the pose and index of
 * the pose graph nodes, which is intended for the use in
 * rendering the current pose graph or searching the loop closure
 * candidate, where the corresponding scan data of the node is not needed
 */
struct NodePosition final
{
    /* Constructor */
    NodePosition(int nodeIdx,
                 const RobotPose2D<double>& pose) :
        mIdx(nodeIdx), mPose(pose) { }
    /* Destructor */
    ~NodePosition() = default;

    /* Comparison (less than) operator */
    inline bool operator<(const NodePosition& other) const
    { return this->mIdx < other.mIdx; }

    /* Index of the node */
    const int                 mIdx;
    /* Robot pose */
    const RobotPose2D<double> mPose;
};

/*
 * EdgeConnection struct represents the indices of the adjacent
 * pose graph nodes and the type (odometry or loop constraint), which is
 * intended for the use in rendering the current pose graph, where
 * the corresponding information matrix is not needed
 */
struct EdgeConnection final
{
    /* Constructor */
    EdgeConnection(int startNodeIdx,
                   int endNodeIdx,
                   bool isOdometry) :
        mStartNodeIdx(startNodeIdx),
        mEndNodeIdx(endNodeIdx),
        mIsOdometry(isOdometry) { }
    /* Destructor */
    ~EdgeConnection() = default;

    /* Index of the start node */
    const int  mStartNodeIdx;
    /* Index of the end node */
    const int  mEndNodeIdx;
    /* Flag to determine whether the edge represents
     * odometry or loop constraint */
    const bool mIsOdometry;
};

/*
 * PoseGraph class represents pose graphs for Graph-Based SLAM
 */
class PoseGraph
{
public:
    /*
     * Node class represents the pose graph nodes same as NodePosition struct
     * but with the associated scan data to build grid maps
     * Poses are the variables in the graph optimization problem
     */
    class Node final
    {
    public:
        /* Constructor */
        Node(int nodeIdx,
             const RobotPose2D<double>& pose,
             const Sensor::ScanDataPtr<double>& scanData) :
            mIdx(nodeIdx), mPose(pose), mScanData(scanData) { }
        
        /* Destructor */
        ~Node() = default;

        /* Copy constructor */
        Node(const Node&) = default;
        /* Copy assignment operator */
        Node& operator=(const Node&) = default;
        /* Move constructor */
        Node(Node&&) = default;
        /* Move assignment operator */
        Node& operator=(Node&&) = default;

        /* Get the index of the node */
        inline int Index() const { return this->mIdx; }

        /* Get the robot pose */
        inline RobotPose2D<double>& Pose() { return this->mPose; }
        inline const RobotPose2D<double>& Pose() const { return this->mPose; }

        /* Get the corresponding scan data */
        inline const Sensor::ScanDataPtr<double>& ScanData() const
        { return this->mScanData; }

    private:
        /* Index of the node */
        int                         mIdx;
        /* Robot pose */
        RobotPose2D<double>         mPose;
        /* Corresponding scan data */
        Sensor::ScanDataPtr<double> mScanData;
    };

    /*
     * Edge class represents pose graph edges (constraints)
     * connecting two adjacent pose graph nodes
     * Relative poses are the constants in the optimization problem
     */
    class Edge final
    {
    public:
        /* Constructor */
        Edge(int startNodeIdx,
             int endNodeIdx,
             const RobotPose2D<double>& relativePose,
             const Eigen::Matrix3d& informationMat) :
            mStartNodeIdx(startNodeIdx),
            mEndNodeIdx(endNodeIdx),
            mRelativePose(relativePose),
            mInformationMat(informationMat) { }

        /* Destructor */
        ~Edge() = default;

        /* Copy constructor */
        Edge(const Edge&) = default;
        /* Copy assignment operator */
        Edge& operator=(const Edge&) = default;
        /* Move constructor */
        Edge(Edge&&) = default;
        /* Move assignment operator */
        Edge& operator=(Edge&&) = default;

        /* Get the index of the start node */
        inline int StartNodeIndex() const { return this->mStartNodeIdx; }
        /* Get the index of the end node */
        inline int EndNodeIndex() const { return this->mEndNodeIdx; }

        /* Get the relative pose between two pose graph nodes */
        inline const RobotPose2D<double>& RelativePose() const
        { return this->mRelativePose; }
        /* Get the information matrix (inverse of the covariance matrix) */
        inline const Eigen::Matrix3d& InformationMatrix() const
        { return this->mInformationMat; }

        /* Check if the edge represents odometric constraint */
        inline bool IsOdometricConstraint() const
        { return this->mEndNodeIdx == this->mStartNodeIdx + 1; }
        /* Check if the edge represents loop closing constraint */
        inline bool IsLoopClosingConstraint() const
        { return !this->IsOdometricConstraint(); }

    private:
        /* Index of the start node */
        int                 mStartNodeIdx;
        /* Index of the end node */
        int                 mEndNodeIdx;
        /* Relative pose between two pose graph nodes */
        RobotPose2D<double> mRelativePose;
        /* Information matrix (inverse of the covariance matrix) */
        Eigen::Matrix3d     mInformationMat;
    };

public:
    /* Constructor */
    PoseGraph() = default;

    /* Destructor */
    ~PoseGraph() = default;

    /* Append new node (new node index is returned) */
    int AppendNode(const RobotPose2D<double>& pose,
                   const Sensor::ScanDataPtr<double>& scanData);
    
    /* Append new edge */
    void AppendEdge(int startNodeIdx,
                    int endNodeIdx,
                    const RobotPose2D<double>& relativePose,
                    const Eigen::Matrix3d& informationMat);
    
    /* Retrieve the list of the pose graph nodes */
    inline const std::vector<Node>& Nodes() const { return this->mNodes; }
    
    /* Retrieve the node of the specified index */
    inline Node& NodeAt(int nodeIdx)
    { return this->mNodes.at(nodeIdx); }
    /* Retrieve the node of the specified index */
    inline const Node& NodeAt(int nodeIdx) const
    { return this->mNodes.at(nodeIdx); }
    
    /* Retrieve the latest node */
    inline Node& LatestNode() { return this->mNodes.back(); }
    /* Retrieve the latest node */
    inline const Node& LatestNode() const { return this->mNodes.back(); }

    /* Retrieve the list of the pose graph edges */
    inline const std::vector<Edge>& Edges() const { return this->mEdges; }

    /* Retrieve the edge of the specified index */
    inline Edge& EdgeAt(int edgeIdx)
    { return this->mEdges.at(edgeIdx); }
    /* Retrieve the edge of the specified index */
    inline const Edge& EdgeAt(int edgeIdx) const
    { return this->mEdges.at(edgeIdx); }

private:
    /* List of the pose graph nodes */
    std::vector<Node> mNodes;
    /* List of the pose graph edges */
    std::vector<Edge> mEdges;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_POSE_GRAPH_HPP */
