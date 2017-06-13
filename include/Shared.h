/**
* This file is part of zSLAM.
*
* Copyright (c) 2016-2017 D. Zúñiga Noël
* For more information see <https://github.com/Peski/zSLAM>
*/

#ifndef Z_SHARED_H
#define Z_SHARED_H

//MRPT
#include <mrpt/poses/CPose3D.h>

//D stuff
#include <DBoW2/DBoW2.h>

//PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

//STL
#include <vector>

class Shared
{
public:

    Shared(std::vector<DBoW2::FBrief::TDescriptor> brief, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, mrpt::poses::CPose3D pose)
        : brief(brief), cloud(cloud), pose(pose)
    {

    }

    std::vector<DBoW2::FBrief::TDescriptor> brief;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    mrpt::poses::CPose3D pose;
};

#endif
