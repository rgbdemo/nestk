#ifndef NESTK_GEOMETRY_POSE_GRAPH_OPTIMIZER_H
# define NESTK_GEOMETRY_POSE_GRAPH_OPTIMIZER_H

#include <vector>
#include <string>
#include <ntk/ntk.h>

/*! Interface for a 6D GraphSLAM optimizer. */
class PoseGraphOptimizer
{
public:
    /*! Adds a new vertex to the graph. The provided 4x4 matrix will be considered as the pose of the new added vertex. It returns the index of the added vertex. */
    virtual int addVertex(ntk::Pose3D& vertexPose) = 0;

    /*! Adds an edge that defines a spatial constraint between the vertices "fromIdx" and "toIdx" with information matrix that determines the weight of the added edge. */
    virtual void addEdge(const int fromIdx, const int toIdx, ntk::Pose3D& relativePose) = 0;

    /*! Calls the graph optimization process to determine the pose configuration that best satisfies the constraints defined by the edges. */
    virtual void optimizeGraph() = 0;

    /*! Returns a vector with all the optimized poses of the graph. */
    virtual void getPoses(std::vector<ntk::Pose3D>& poses) = 0;

    /*! Saves the graph to file. */
    virtual void saveGraph(std::string fileName) = 0;
};

#endif
