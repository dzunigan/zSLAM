/** AUTHOR: David Zúñiga */

#ifndef SHARED_OBJ_H
#define SHARED_OBJ_H

#include <mrpt/synch/CSemaphore.h>
#include <mrpt/poses/CPose3D.h>

#include <mrpt/obs/CObservation3DRangeScan.h>

#include <DBoW2/DBoW2.h>

#include <queue>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "Dataset.h"

class SharedObj
{
public:

    SharedObj(const BriefVocabulary &v)
        : voc(NULL), sync(1, 1)
    {
        voc = new BriefVocabulary(v);
    }

    virtual ~SharedObj()
    {
        delete voc;
    }

    mrpt::synch::CSemaphore sync;
    BriefVocabulary *voc;
    Dataset rawlog;

    std::queue< std::vector<DBoW2::FBrief::TDescriptor> > q_d;
    std::queue<pcl::PointCloud<pcl::PointXYZ>::Ptr> q_c;
    std::queue<mrpt::poses::CPose3D> q_o;

    std::vector<mrpt::obs::CObservation3DRangeScanPtr> v_o;

    int downsample;
    int ctf_levels;

    int th;
    int max_k;

    float beta;

    float f;
    float alpha;

    DVision::BRIEF brief;
};

#endif
