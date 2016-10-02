/**
* This file is part of zSLAM.
*
* Copyright (c) 2016 D. Zúñiga Noël
* For more information see <https://github.com/Peski/zSLAM>
*/

#ifndef Z_RANSAC_3D_H
#define Z_RANSAC_3D_H

//STL
#include <vector>

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/transformation_estimation_svd.h>

//D Stuf
//#include <DUtils/Profiler.h>
#include <DUtils/Random.h>

bool ransac3d(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_src, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_tgt, const pcl::Correspondences &correspondences, const float t, pcl::Correspondences &inliers, Eigen::Matrix4f &transform, int min_points = 12, double p = 0.99, int MAX_ITS = 500)//, DUtils::Profiler &profiler)
{
//    profiler.profile("RANSAC_3D");

    const int M = 3; //model points
    int N = correspondences.size();

    if (N < min_points)
        return false;

    if (min_points < M)
        min_points = M;

    const double log1p = log(1-p);
    int max_its;
    if(N == min_points)
        max_its = 1;
    else
    {
        max_its = int(log1p / log(1 - pow(double(min_points)/double(N), min_points)));
        if(max_its > MAX_ITS || max_its <= 0) max_its = MAX_ITS;
        // < 0 in case of wrapping around because of overflow
        // this is to avoid numerical problems
        if(max_its == std::numeric_limits<int>::max()) max_its--;
    }

    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> estimator(false);
    pcl::registration::CorrespondenceRejectorDistance rejector;
    rejector.setMaximumDistance(t);

    rejector.setInputTarget<pcl::PointXYZ>(cloud_tgt);

    DUtils::Random::SeedRandOnce();

    std::vector<int> all_i_available;
    all_i_available.reserve(N);

    for(int i = 0; i < N; ++i)
        all_i_available.push_back(i);

    std::vector<int> i_available;
    i_available.reserve(N);

    std::vector<int> indices_src;
    indices_src.resize(M);

    std::vector<int> indices_tgt;
    indices_tgt.resize(M);

    inliers.clear();
    pcl::Correspondences best;

    transform = Eigen::Matrix4f::Identity();
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    rejector.setInputSource<pcl::PointXYZ>(transformed_cloud);

    for (int it = 0; it < max_its; ++it)
    {
        // get model points
        i_available = all_i_available;

        for(short i = 0; i < M; ++i)
        {
          int idx = DUtils::Random::RandomInt(0, i_available.size()-1);

          indices_src[i] = correspondences[i_available[idx]].index_query;
          indices_tgt[i] = correspondences[i_available[idx]].index_match;

          i_available[idx] = i_available.back();
          i_available.pop_back();
        }

        // get model
        estimator.estimateRigidTransformation(*cloud_src, indices_src, *cloud_tgt, indices_tgt, transform);
        pcl::transformPointCloud(*cloud_src, *transformed_cloud, transform);

        rejector.getRemainingCorrespondences(correspondences, inliers);

        if (inliers.size() > best.size() && inliers.size() > min_points)
        {
            best = inliers;
            int its = int(log1p / log(1 - pow(double(inliers.size())/double(N), M)));
            if(0 <= its && its < max_its) max_its = its;
        }
    }

    if (best.size() >= min_points)
    {
        inliers = best;

        //return transformation with all inliers
        indices_src.resize(inliers.size());
        indices_tgt.resize(inliers.size());

        for(short i = 0; i < inliers.size(); ++i)
        {
          indices_src[i] = inliers[i].index_query;
          indices_tgt[i] = inliers[i].index_match;
        }

        estimator.estimateRigidTransformation(*cloud_src, indices_src, *cloud_tgt, indices_tgt, transform);

        return true;
    }

    return false;

//    profiler.stop();
}

#endif
