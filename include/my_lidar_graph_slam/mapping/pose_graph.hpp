
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
 * PoseGraph class represents pose graphs for Graph-Based SLAM
 */
class PoseGraph
{
public:
    /*
     * Node class represents pose graph nodes
     * poses are variables in the graph optimization problem
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

        /* Copy constructor (disabled) */
        Node(const Node&) = delete;
        /* Copy assignment operator (disabled) */
        Node& operator=(const Node&) = delete;
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
     * relative poses are constants in the optimization problem
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

        /* Copy constructor (disabled) */
        Edge(const Edge&) = delete;
        /* Copy assignment operator (disabled) */
        Edge& operator=(const Edge&) = delete;
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