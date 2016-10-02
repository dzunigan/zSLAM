/**
* This file is part of zSLAM.
*
* Copyright (c) 2016 D. Zúñiga Noël
* For more information see <https://github.com/Peski/zSLAM>
*/

#ifndef Z_UTIL_H
#define Z_UTIL_H

#include <mrpt/system/filesystem.h>

void wait()
{
    std::cout << "Press enter to continue" << std::endl;
    std::getchar();
}

bool cmp (cv::KeyPoint k1, cv::KeyPoint k2)
{
    return k1.response > k2.response;
}

void filter(std::vector<cv::KeyPoint> &k, int n)
{
    std::sort(k.begin(), k.end(), cmp);

    if (n < k.size())
        k.resize(n);
}

void projectTo(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const mrpt::obs::CObservation3DRangeScanPtr &obs, const std::vector<cv::KeyPoint> &points)
{
    const mrpt::math::CMatrix range = obs->rangeImage;

    float inv_fx = 1.0f / obs->cameraParams.fx();
    float inv_fy = 1.0f / obs->cameraParams.fy();
    float cx = static_cast<float> (obs->cameraParams.cx());
    float cy = static_cast<float> (obs->cameraParams.cy());

    cloud->clear();
    cloud->reserve(points.size());

    for (int i = 0; i < points.size(); ++i)
    {
        cv::Point2f p = points[i].pt;

        int x = static_cast<int> (p.x);
        int y = static_cast<int> (p.y);
        float d = range(y, x);

        cloud->push_back(pcl::PointXYZ (d, (cx - x) * d * inv_fx, (cy - y) * d * inv_fy));
    }
}

g2o::Isometry3D fromCPose3D(const mrpt::poses::CPose3D &p)
{
    mrpt::math::CMatrixDouble44 m;
    p.getHomogeneousMatrix(m);

    g2o::Isometry3D t(m);
    return t;
}

mrpt::poses::CPose3D toCPose3D(const g2o::Isometry3D &t)
{
    mrpt::math::CMatrixDouble44 m(t.matrix());

    mrpt::poses::CPose3D p(m);
    return p;
}

void writeTrajectory(const std::string &filename, const std::vector<double> &timestamps, const std::vector<g2o::Isometry3D> &poses)
{
    std::fstream f(filename.c_str(), std::ios::out);

    assert(timestamps.size() == poses.size());

    mrpt::math::CQuaternionDouble quat;
    mrpt::poses::CPose3D pose;
    char time[24];

    mrpt::poses::CPose3D transf;
    transf.setFromValues(0,0,0,0.5*M_PI, -0.5*M_PI, 0);

    for (int i = 0; i < poses.size(); ++i)
    {
        sprintf(time,"%.04f", timestamps[i]);
        f << time << " ";

        pose = toCPose3D(poses[i]);
        f << pose[0] << " ";
        f << pose[1] << " ";
        f << pose[2] << " ";

        pose = pose - transf;
        pose.getAsQuaternion(quat);
        f << quat(2) << " ";
        f << quat(3) << " ";
        f << -quat(1) << " ";
        f << -quat(0) << endl;
    }
}

void getFreeFile(std::string prefix, std::string &filename)
{
    // Open file, find the first free file-name.
    char	aux[100];
    int     nFile = 0;
    bool    free_name = false;

    std::string file = "./data/exp/";
    file.append(prefix);

    std::string tmp;
    while (!free_name)
    {
        nFile++;

        sprintf(aux, "_%03u.txt", nFile );
        tmp = file;
        tmp.append(aux);
        free_name = !mrpt::system::fileExists(tmp.c_str());
    }

    filename = tmp;
//    std::cout << " Saving results to file: " << filename << std::endl;
}

#endif
