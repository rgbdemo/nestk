/**
 * Copyright (C) 2013 ManCTL SARL <contact@manctl.com>
 * Copyright (C) 2012 Cristobal Belles <playvision.belles@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Cristobal Belles <playvision.belles@gmail.com>
 */

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
