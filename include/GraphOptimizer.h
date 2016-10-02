/**
* This file is part of zSLAM.
*
* Copyright (c) 2016 D. Zúñiga Noël
* For more information see <https://github.com/Peski/zSLAM>
*/

#ifndef Z_GRAPH_OPTIMIZER
#define Z_GRAPH_OPTIMIZER

#include <vector>
#include <string>

#include <Eigen/Core>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/types/slam3d/types_slam3d.h>

namespace Eigen {
    typedef Eigen::Matrix<double, 6, 6> Matrix6d;
}

class GraphOptimizer
{
private:
    int n_vertices;
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType *linearSolver;
    g2o::BlockSolver_6_3 *solver_ptr;
    g2o::OptimizationAlgorithmLevenberg *solver;
public:
  GraphOptimizer(bool verbose = true);
  /*!Adds a new vertex to the graph. The provided 4x4 matrix will be considered as the pose of the new added vertex. It returns the index of the added vertex.*/
  int addVertex(const g2o::Isometry3D &pose);
  /*!Adds an edge that defines a spatial constraint between the vertices "fromId" and "toId" with Identity information matrix of the added edge.*/
  void addEdge(const int fromId, const int toId, const g2o::Isometry3D &rel_pose, const Eigen::Matrix6d &information);
  /*!Adds an edge tha defines a loop closure between the vertices "fromId" and "toId", asigning "new_pose" pose to vertex "toId".*/
  void addLoop(const int fromId, const int toId, const g2o::Isometry3D &rel_pose, const g2o::Isometry3D &new_pose);
  /*!Calls the graph optimization process to determine the pose configuration that best satisfies the constraints defined by the edges.*/
  void optimizeGraph();
  /*!Returns a vector with all the optimized poses of the graph.*/
  void getPoses(std::vector<g2o::Isometry3D> &poses);
  /*!Returns the vale of the "id" state variable.*/
  void getPose(const int id, g2o::Isometry3D &pose);
  /*!Returns the number of nodes in the graph.*/
  int size();
  /*!Saves the graph to file.*/
  void saveGraph(const std::string fileName);
};

#endif
