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

#ifndef POSE_GRAPH_OPTIMIZER_G2O_H
# define POSE_GRAPH_OPTIMIZER_G2O_H

#include "pose_graph_optimizer.h"

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

/*!
 * Encapsulates the G2O library to perform 6D graph optimization.
 */
class PoseGraphOptimizerG2O : public PoseGraphOptimizer
{
public:
    PoseGraphOptimizerG2O();

    /*! Adds a new vertex to the graph. The provided 4x4 matrix will be considered as the pose of the new added vertex. It returns the index of the added vertex. */
    int addVertex(ntk::Pose3D& vertexPose);

    /*! Adds an edge that defines a spatial constraint between the vertices "fromIdx" and "toIdx" with information matrix that determines the weight of the added edge. */
    //void addEdge(const int fromIdx,const int toIdx,Eigen::Matrix4f& relPose,Eigen::Matrix<double,6,6>& infMatrix);
    void addEdge(const int fromIdx, const int toIdx, ntk::Pose3D& relativePose);

    /*! Calls the graph optimization process to determine the pose configuration that best satisfies the constraints defined by the edges. */
    void optimizeGraph();

    /*! Returns a vector with all the optimized poses of the graph. */
    //void getPoses(std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f > >&);
    void getPoses(std::vector<ntk::Pose3D>& poses);

    /*! Saves the graph to file. */
    void saveGraph(std::string fileName);

private:
    int vertexIdx;
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType* linearSolver;
    g2o::BlockSolverX* solver_ptr;
};

#endif
