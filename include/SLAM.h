/** AUTHOR: David Zúñiga */

#ifndef Z_SLAM_H
#define Z_SLAM_H

#include <mrpt/synch/CSemaphore.h>
#include <mrpt/poses/CPose3D.h>

#include <mrpt/system/threads.h>

#include <mrpt/obs/CObservation3DRangeScan.h>

#include <mrpt/hwdrivers/COpenNI2Sensor.h>

#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/CPointCloudColoured.h>

#include <DBoW2/DBoW2.h>

#include <vector>
#include <queue>
#include <algorithm>
#include <stdio.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "Difodo.h"
#include "TemplatedKeyframeSelection.h"

typedef TemplatedKeyframeSelection
  <DBoW2::FBrief::TDescriptor, DBoW2::FBrief> BriefKeyframeSelection;

class SLAM
{
public:

    SLAM(const BriefVocabulary &v, mrpt::hwdrivers::COpenNI2Sensor *sensor, int downsample, int ctf_levels, int th, int max_k, float beta, float f, float alpha)
        : voc(NULL), m_sensor(sensor), sync(1, 1, "ZSLAM"), m_downsample(downsample), m_ctf_levels(ctf_levels), m_th(th), m_beta(beta), m_alpha(alpha)
    {
        voc = new BriefVocabulary(v);
    }

    virtual ~SLAM()
    {
        delete voc;
    }

    void run();

    DVision::BRIEF brief;

protected:

    static void filter(std::vector<cv::KeyPoint> &k, int n);
    static void projectTo(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const mrpt::obs::CObservation3DRangeScanPtr &obs, const std::vector<cv::KeyPoint> &points);

    static void dummy_tracking( SLAM *obj );
    static void dummy_mapping( SLAM *obj );

    void thread_tracking( );
    void thread_mapping( );

    mrpt::synch::CSemaphore sync;
    BriefVocabulary *voc;

    mrpt::hwdrivers::COpenNI2Sensor *m_sensor;

    std::queue< std::vector<DBoW2::FBrief::TDescriptor> > q_d;
    std::queue<pcl::PointCloud<pcl::PointXYZ>::Ptr> q_c;
    std::queue<mrpt::poses::CPose3D> q_o;

    std::queue<mrpt::obs::CObservation3DRangeScanPtr> q_obs;

    std::vector<mrpt::obs::CObservation3DRangeScanPtr> v_o;

    int m_downsample;
    int m_ctf_levels;

    int m_th;
    int m_max_k;

    float m_beta;

    float m_f;
    float m_alpha;

private:

    static bool cmp (cv::KeyPoint k1, cv::KeyPoint k2);

    mrpt::system::TThreadHandle		m_thread_tracking;
    mrpt::system::TThreadHandle		m_thread_mapping;

};

void SLAM::run()
{

    std::cout << "running..." << std::endl;

    m_thread_tracking = mrpt::system::createThread(dummy_tracking, this);
    m_thread_mapping = mrpt::system::createThread(dummy_mapping, this);

    mrpt::system::joinThread(m_thread_tracking);
    mrpt::system::joinThread(m_thread_mapping);
}

void SLAM::dummy_tracking( SLAM *obj )
{
    obj->thread_tracking();
}

void SLAM::dummy_mapping( SLAM *obj )
{
    obj->thread_mapping();
}

void SLAM::thread_tracking( )
{
    //Tracking
    Difodo odo(m_downsample, m_ctf_levels);
    BriefKeyframeSelection keyframe(*voc, m_beta);

    mrpt::poses::CPose3D last_pose;
    last_pose.setFromValues(0.0, 0.0, 0.0);

    mrpt::obs::CObservation3DRangeScanPtr obs = mrpt::obs::CObservation3DRangeScan::Create();
    bool bObs = false, bError = true;
    m_sensor->getNextObservation(*obs, bObs, bError);
    bool end = (!bObs || bError);
    while (!end)
    {
//        #pragma omp parallel sections
        {
//            #pragma omp section
            {
                //Processing...
                odo.compute(obs);

                cv::Mat rgb_image(obs->intensityImage.getAs<IplImage>());

                cv::Mat img_gray;
                cv::cvtColor(rgb_image, img_gray, CV_BGR2GRAY);

                std::vector<cv::KeyPoint> gray_k;
                std::vector<DBoW2::FBrief::TDescriptor> gray_d;

                cv::FAST(img_gray, gray_k, m_th, true);
                filter(gray_k, m_max_k);

                brief.compute(img_gray, gray_k, gray_d, true);

                bool is_keyframe = keyframe.process(gray_d);
                if (is_keyframe)
                {
                    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
                    projectTo(cloud, obs, gray_k);

                    //Pose increment (or edge):
                    mrpt::poses::CPose3D inc(mrpt::poses::UNINITIALIZED_POSE);
                    inc.inverseComposeFrom(odo.cam_pose, last_pose);

                    sync.waitForSignal();
//                    q_d.push(gray_d);
//                    q_c.push(cloud);
//                    q_o.push(inc);
                    q_o.push(inc);
//                    v_o.push_back(obs);
                    q_obs.push(obs);
                    sync.release();

                    last_pose = odo.cam_pose;
                    obs = mrpt::obs::CObservation3DRangeScan::Create();
                }
            }

            mrpt::system::sleep(66);

//            #pragma omp section
            {
                m_sensor->getNextObservation(*obs, bObs, bError);
                end = (!bObs || bError);
            }
        }
    }
}

void SLAM::thread_mapping( )
{
    mrpt::gui::CDisplayWindow3D window;
    mrpt::opengl::COpenGLScenePtr scene;
    window.setWindowTitle("Map");
    window.resize(800,600);
    window.setPos(500,50);
    window.setCameraZoom(5);
    window.setCameraAzimuthDeg(180);
    window.setCameraElevationDeg(5);

    mrpt::poses::CPose3D last_pose, cam_pose, inc;
    last_pose.setFromValues(0.0, 0.0, 0.0);

    for(;;)
    {
        std::cout << "Loop..." << std::endl;
        mrpt::obs::CObservation3DRangeScanPtr obs;
        int n;

        sync.waitForSignal();
        assert(q_o.size() == q_obs.size());
        n = q_obs.size();
        if (n > 0)
        {
            inc = q_o.front();
            q_o.pop();

            obs = q_obs.front();
            q_obs.pop();
        }
        sync.release();

        if (n > 0)
        {
            cam_pose = last_pose + inc;

            scene = window.get3DSceneAndLock();

            mrpt::opengl::CPointCloudColouredPtr gl_points = mrpt::opengl::CPointCloudColoured::Create();
            gl_points->enablePointSmooth(true);
            gl_points->setPointSize(2);
            scene->insert(gl_points);

            obs->setSensorPose(cam_pose);
            obs->project3DPointsFromDepthImageInto(*gl_points, true);

            window.unlockAccess3DScene();
            window.repaint();

            last_pose = cam_pose;
        }
        else
            mrpt::system::sleep(17);
    }
}

void SLAM::filter(std::vector<cv::KeyPoint> &k, int n)
{
    std::sort(k.begin(), k.end(), cmp);

    if (n < k.size())
        k.resize(n);
}

void SLAM::projectTo(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const mrpt::obs::CObservation3DRangeScanPtr &obs, const std::vector<cv::KeyPoint> &points)
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

//        cloud->push_back(pcl::PointXYZ ((x - cx) * d * inv_fx, (y - cy) * d * inv_fy, d));
        cloud->push_back(pcl::PointXYZ (d, (cx - x) * d * inv_fx, (cy - y) * d * inv_fy));
    }
}

bool SLAM::cmp (cv::KeyPoint k1, cv::KeyPoint k2)
{
    return k1.response > k2.response;
}

#endif
