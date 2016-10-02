/**
* This file is part of zSLAM.
*
* Copyright (c) 2016 D. Zúñiga Noël
* For more information see <https://github.com/Peski/zSLAM>
*/

#include "GraphOptimizer.h"

GraphOptimizer::GraphOptimizer(bool verbose)
{
    optimizer.setVerbose(verbose);
    linearSolver = new g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>();
    solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
    solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    solver->setUserLambdaInit(1e-10);
    optimizer.setAlgorithm(solver);

    //Set the number of vertices to 0
    n_vertices = 0;
}

int GraphOptimizer::addVertex(const g2o::Isometry3D &pose)
{
    //Set up node
    g2o::VertexSE3 *v_se3 = new g2o::VertexSE3();
    v_se3->setId(n_vertices);
    v_se3->setMarginalized(false);

    //v_se3->estimate() = pose;
    v_se3->setEstimate(pose);

    if (n_vertices == 0)
    { //First node
        v_se3->setFixed(true);
    }

    // add to optimizer
    optimizer.addVertex(v_se3);

    //Update the number of vertices
    n_vertices++;

    //Return the added vertex id
    return (n_vertices - 1);
}

void GraphOptimizer::addEdge(const int fromId, const int toId, const g2o::Isometry3D &rel_pose, const Eigen::Matrix6d &information)
{
    g2o::EdgeSE3 *e_se3 = new g2o::EdgeSE3;
    e_se3->setVertex(0, optimizer.vertex(fromId));
    e_se3->setVertex(1, optimizer.vertex(toId));
    e_se3->setMeasurement(rel_pose);

    //Set the information matrix
    e_se3->setInformation(information);

    optimizer.addEdge(e_se3);
}

void GraphOptimizer::addLoop(const int fromId, const int toId, const g2o::Isometry3D &rel_pose, const g2o::Isometry3D &new_pose)
{

    g2o::VertexSE3 *v_src = static_cast<g2o::VertexSE3*>(optimizer.vertex(fromId));
    v_src->setFixed(true);

    g2o::VertexSE3 *v_tgt = static_cast<g2o::VertexSE3*>(optimizer.vertex(toId));
    v_tgt->setEstimate(new_pose);
    v_tgt->setFixed(true);

    g2o::EdgeSE3 *e_se3 = new g2o::EdgeSE3;
    e_se3->setVertex(0, v_src);
    e_se3->setVertex(1, v_tgt);
    e_se3->setMeasurement(rel_pose);

    //Set the information matrix to identity
    e_se3->setInformation(Eigen::Matrix6d::Identity());

    optimizer.addEdge(e_se3);
}

void GraphOptimizer::optimizeGraph()
{
    optimizer.initializeOptimization();
    optimizer.computeInitialGuess();
    optimizer.computeActiveErrors();
    optimizer.optimize(100);
}

void GraphOptimizer::getPoses(std::vector< g2o::Isometry3D > &poses)
{
    poses.clear();
    poses.resize(n_vertices);

    for(int id = 0; id < n_vertices; ++id)
    {
        //Get the Isometry3D estimate from the G2O vertex pose
        g2o::VertexSE3* v_se3 = static_cast<g2o::VertexSE3*>(optimizer.vertex(id));
        g2o::Isometry3D optimized_pose = v_se3->estimate();

        //Set the optimized pose to the vector of poses
        poses[id] = optimized_pose;
    }
}

void GraphOptimizer::getPose(const int id, g2o::Isometry3D &pose)
{
    if (id >= 0 && id < n_vertices)
    {
        //Get the Isometry3D estimate from the G2O vertex pose
        g2o::VertexSE3* v_se3 = static_cast<g2o::VertexSE3*>(optimizer.vertex(id));
        pose = v_se3->estimate();
    }
}

int GraphOptimizer::size()
{
    return n_vertices;
}

void GraphOptimizer::saveGraph(const std::string fileName)
{
        //Save the graph to file
        optimizer.save(fileName.c_str(), 0);
}
