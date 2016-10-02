/**
* This file is part of zSLAM.
*
* Copyright (c) 2016 D. Zúñiga Noël
* For more information see <https://github.com/Peski/zSLAM>
*/

#include "Difodo.h"

#include <mrpt/utils/adapters.h>

#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/opengl/CSetOfLines.h>

using namespace std;
using namespace mrpt;

using namespace mrpt::math;
using namespace mrpt::obs;
using namespace mrpt::opengl;

Difodo::Difodo(unsigned int downsample, unsigned int ctf_levels) : mrpt::vision::CDifodo(),
       m_initialized(false), m_downsample(downsample), m_ctf_levels(ctf_levels)
{
   reset();
}

void Difodo::loadFrame()
{
    return;
}

void Difodo::loadFrame(const CObservation3DRangeScanPtr &obs)
{
    const CMatrix range = obs->rangeImage;
    const unsigned int height = range.getRowCount();
    const unsigned int width = range.getColCount();

    for (unsigned int j = 0; j < cols; j++)
        for (unsigned int i = 0; i < rows; i++)
        {
            const float z = range(height-downsample*i-1, width-downsample*j-1);
            if (z < 4.5f)	depth_wf(i, j) = z;
            else			depth_wf(i, j) = 0.f;
        }
}

void Difodo::initialize()
{

//			    Load Config
//=============================================================
    fovh = M_PI*62.5/180.0;	//Larger FOV because depth is registered with color
    fovv = M_PI*48.5/180.0;
    cam_mode = 1;
    fast_pyramid = false;
    downsample = m_downsample;
    ctf_levels = m_ctf_levels;
    cols = 640/(cam_mode*downsample);
    rows = 480/(cam_mode*downsample);
    fps = 30;

    //			Resize matrices and adjust parameters
    //=========================================================
    width = 640/(cam_mode*downsample);
    height = 480/(cam_mode*downsample);

    //Resize pyramid
    const unsigned int pyr_levels = round(log(float(width/cols))/log(2.f)) + ctf_levels;
    depth.resize(pyr_levels);
    depth_old.resize(pyr_levels);
    depth_inter.resize(pyr_levels);
    depth_warped.resize(pyr_levels);
    xx.resize(pyr_levels);
    xx_inter.resize(pyr_levels);
    xx_old.resize(pyr_levels);
    xx_warped.resize(pyr_levels);
    yy.resize(pyr_levels);
    yy_inter.resize(pyr_levels);
    yy_old.resize(pyr_levels);
    yy_warped.resize(pyr_levels);
    transformations.resize(pyr_levels);

    for (unsigned int i = 0; i<pyr_levels; i++)
    {
        unsigned int s = pow(2.f,int(i));
        cols_i = width/s; rows_i = height/s;
        depth[i].resize(rows_i, cols_i);
        depth_inter[i].resize(rows_i, cols_i);
        depth_old[i].resize(rows_i, cols_i);
        depth[i].assign(0.0f);
        depth_old[i].assign(0.0f);
        xx[i].resize(rows_i, cols_i);
        xx_inter[i].resize(rows_i, cols_i);
        xx_old[i].resize(rows_i, cols_i);
        xx[i].assign(0.0f);
        xx_old[i].assign(0.0f);
        yy[i].resize(rows_i, cols_i);
        yy_inter[i].resize(rows_i, cols_i);
        yy_old[i].resize(rows_i, cols_i);
        yy[i].assign(0.0f);
        yy_old[i].assign(0.0f);
        transformations[i].resize(4,4);

        if (cols_i <= cols)
        {
            depth_warped[i].resize(rows_i,cols_i);
            xx_warped[i].resize(rows_i,cols_i);
            yy_warped[i].resize(rows_i,cols_i);
        }
    }

    //Resize matrix that store the original depth image
    depth_wf.setSize(height,width);

//			    Initialize Scene
//=============================================================

////    global_settings::OCTREE_RENDER_MAX_POINTS_PER_NODE = 1000000;
//    window.setWindowTitle("Odometry");
//    window.resize(800,600);
//    window.setPos(500,50);
//    window.setCameraZoom(5);
//    window.setCameraAzimuthDeg(180);
//    window.setCameraElevationDeg(5);
////    window.setCameraPointingToPoint(0,0,0);
////    window.setCameraPointingToPoint(0,0,1);

//    scene = window.get3DSceneAndLock();

//    //Dif Odometry
//    CSetOfLinesPtr traj_lines_odo = CSetOfLines::Create();
//    traj_lines_odo->setLocation(0,0,0);
//    traj_lines_odo->setColor(0,0.6,0);
//    traj_lines_odo->setLineWidth(6);
//    scene->insert(traj_lines_odo);

//    CPointCloudPtr traj_points_odo = CPointCloud::Create();
//    traj_points_odo->setColor(0,0.6,0);
//    traj_points_odo->setPointSize(4);
//    traj_points_odo->enablePointSmooth(1);
//    scene->insert(traj_points_odo);

//    window.unlockAccess3DScene();

    m_initialized = true;
}

//void Difodo::updateScene()
//{
//    scene = window.get3DSceneAndLock();

//    opengl::CPointCloudColouredPtr cam_points = opengl::CPointCloudColoured::Create();
//    cam_points->enablePointSmooth(true);
//    cam_points->setPointSize(2);
//    scene->insert(cam_points);

//    obs3D->setSensorPose(cam_pose);
//    obs3D->project3DPointsFromDepthImageInto(*cam_points, true);

//    //Difodo trajectory
//    CSetOfLinesPtr traj_lines_odo = scene->getByClass<CSetOfLines>(0);
//    traj_lines_odo->appendLine(cam_oldpose.x(), cam_oldpose.y(), cam_oldpose.z(), cam_pose.x(), cam_pose.y(), cam_pose.z());

//    CPointCloudPtr traj_points_odo = scene->getByClass<CPointCloud>(0);
//    traj_points_odo->insertPoint(cam_pose.x(), cam_pose.y(), cam_pose.z());

//    window.unlockAccess3DScene();
//    window.repaint();
//}

void Difodo::compute()
{
    if (m_initialized)
    { //First observation
        if (fast_pyramid)	buildCoordinatesPyramidFast();
        else				buildCoordinatesPyramid();

        cam_pose.setFromValues(0, 0, 0);
        cam_oldpose = cam_pose;

        m_initialized = false;
    }
    else //
        odometryCalculation();
}

void Difodo::compute(const CObservation3DRangeScanPtr &obs)
{
    loadFrame(obs);

    if (m_initialized)
    { //First observation
        if (fast_pyramid)	buildCoordinatesPyramidFast();
        else				buildCoordinatesPyramid();

        cam_pose.setFromValues(0.0, 0.0, 0.0);
        cam_oldpose = cam_pose;

        m_initialized = false;
    }
    else //
        odometryCalculation();
}

void Difodo::reset()
{
    m_initialized = false;
    initialize();
}
