/**
* This file is part of zSLAM.
*
* Copyright (c) 2016-2017 D. Zúñiga Noël
* For more information see <https://github.com/Peski/zSLAM>
*/

#include "Shared.h"

Shared::Shared(std::vector<DBoW2::FBrief::TDescriptor> brief, pcl::PointCloud::Ptr cloud, mrpt::poses::CPose3D pose)
    : brief(brief), cloud(cloud), pose(pose)
{

}
