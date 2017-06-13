/**
* This file is part of zSLAM.
*
* Copyright (c) 2016-2017 D. Zúñiga Noël
* For more information see <https://github.com/Peski/zSLAM>
*/

//MRPT
#include <mrpt/poses/CPose3D.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/utils/CConfigFile.h>

//For 3D dense representation
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/opengl/CSetOfLines.h>

//D stuff
#include <DBoW2/DBoW2.h>
#include <DUtils/DUtils.h>

//PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

//STL
//#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <stdio.h>

//POSIX
#include <pthread.h>
#include <time.h>

//OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "TemplatedLoopDetector3D.h"
#include "TemplatedKeyframeSelection.h"

#include "Difodo.h"
#include "GraphOptimizer.h"
#include "Input.h"
#include "Shared.h"

#include "util.h"
#include "config.h"

/// BRIEF-based
typedef TemplatedLoopDetector3D
  <DBoW2::FBrief::TDescriptor, DBoW2::FBrief> BriefLoopDetector;
typedef TemplatedKeyframeSelection
  <DBoW2::FBrief::TDescriptor, DBoW2::FBrief> BriefKeyframeSelection;

struct args_t
{
    int max_k;
    int skip;

    int downsample;
    int ctf_levels;

    int th;

    Input *input;

    DVision::BRIEF *brief;
    BriefKeyframeSelection *keyframe;

    BriefLoopDetector *detector;
    mrpt::gui::CDisplayWindow3D *window;

    pthread_mutex_t *sync;
    pthread_cond_t *cond;

    std::queue<Shared> *q;
    std::vector<mrpt::obs::CObservation3DRangeScanPtr> *v_o;
};

volatile static int cnt;

void *tracking(void *arg);

void *mapping(void *arg);

int main(int argc, char** argv)
{
    std::cout << "Copyright (c) 2016-2017, D. Zúñiga Noël" << std::endl;
    std::cout << std::endl;

    std::string config_path = "./data/config"; // Default

    // Input:
    Input input;

    if (argc >= 2)
#if WITH_DATASET
    // Dataset:
    // ----------------------------------
    {
        std::string dataset = argv[1];
        input.loadDataset(dataset);

        std::cout << "Using dataset: " << dataset << std::endl;
    }
    else
    {
        std::cout << "Usage: [DATASET] [CONFIG]" << std::endl;

        return 0;
    }
#else
    // Open sensor:
    // ----------------------------------
    {
        unsigned sensor_id;
        input.openSensor(sensor_id);

        std::cout << "Using device: " << sensor_id << std::endl;
    }
    else
    {
        std::cout << "Usage: [DEVICE] [CONFIG]" << std::endl;

        return 0;
    }
#endif

    if (argc > 2)
        config_path = argv[2];

    std::cout << "Using config: " << config_path << std::endl;
    std::cout << std::endl;

    mrpt::utils::CConfigFile config(config_path);

    //GENERAL
    int max_k = config.read_int("GENERAL", "max_k", 300);
    int skip = config.read_int("GENERAL", "skip", 1);

    std::cout << "GENERAL" << std::endl;
    std::cout << "max_k: " << max_k << std::endl;
    std::cout << "skip: " << skip << std::endl;

    //DIFODO
    int downsample = config.read_int("DIFODO", "downsample", 4);
    int ctf_levels = config.read_int("DIFODO", "ctf_levels", 5);

    std::cout << "DIFODO" << std::endl;
    std::cout << "downsample: " << downsample << std::endl;
    std::cout << "ctf_levels: " << ctf_levels << std::endl;

    //FEATURES
    int th = config.read_int("FEATURES", "th", 10);
    std::string pattern = config.read_string("FEATURES", "pattern", "", true);

    std::cout << "FEATURES" << std::endl;
    std::cout << "th: " << th << std::endl;
    std::cout << "pattern: " << pattern << std::endl;

    //LOOP_DETECTION
    std::string vocabulary = config.read_string("LOOP_DETECTION", "vocabulary", "", true);
    float f = config.read_float("LOOP_DETECTION", "f", 1.f);
    float alpha = config.read_float("LOOP_DETECTION", "alpha", 0.3f);

    std::cout << "LOOP_DETECTION" << std::endl;
    std::cout << "vocabulary: " << vocabulary << std::endl;
    std::cout << "f: " << f << std::endl;
    std::cout << "alpha: " << alpha << std::endl;

    //KEYFRAME_SELECTION
    float beta = config.read_float("KEYFRAME_SELECTION", "beta", 0.8f);

    std::cout << "KEYFRAME_SELECTION" << std::endl;
    std::cout << "beta: " << beta << std::endl;

    // Load pattern
    cv::String pattern_file(pattern);
    cv::FileStorage fs(pattern_file, cv::FileStorage::READ);
    if(!fs.isOpened()) throw std::string("Could not open pattern file");

    std::vector<int> x1, y1, x2, y2;
    fs["x1"] >> x1;
    fs["x2"] >> x2;
    fs["y1"] >> y1;
    fs["y2"] >> y2;

    // Load the vocabulary to use
    std::cout << "Loading vocabulary..." << std::endl;
    BriefVocabulary voc(vocabulary);
    std::cout << "... done." << std::endl;

    args_t *args = new args_t;

    args->max_k = max_k;
    args->skip = skip;

    args->downsample = downsample;
    args->ctf_levels = ctf_levels;

    args->th = th;

    // Tracking
    DVision::BRIEF brief;
    BriefKeyframeSelection keyframe(voc, beta);
    brief.importPairs(x1, y1, x2, y2);

    args->input = &input;
    args->brief = &brief;
    args->keyframe = &keyframe;

    // Mapping
    BriefLoopDetector::Parameters params(480, 640, f, true, alpha, 0, GEOM_DI, 4);
    BriefLoopDetector detector(voc, params);

    args->detector = &detector;

    //For 3D dense reconstruction
#if WITH_3DREP
    mrpt::gui::CDisplayWindow3D window;
    window.setWindowTitle("Map");
    window.resize(800,600);
    window.setPos(0,0);
    window.setCameraZoom(16);
    window.setCameraAzimuthDeg(0);
    window.setCameraElevationDeg(45);

    args->window = &window;

    mrpt::opengl::COpenGLScenePtr scene;
#endif

    // Thread synch
    pthread_mutex_t sync = PTHREAD_MUTEX_INITIALIZER;
    pthread_cond_t cond = PTHREAD_COND_INITIALIZER;

    args->sync = &sync;
    args->cond = &cond;

    // Shared between T&M threads
    std::queue<Shared> q;
    //For 3D dense reconstruction
    std::vector<mrpt::obs::CObservation3DRangeScanPtr> v_o;

    args->q = &q;
    args->v_o = &v_o;

    //Final trajectory
    std::vector<g2o::Isometry3D> opt;

    //User-fired termination
    bool finish = false;

    std::cout << "Running..." << std::endl;
    cnt = true;

    pthread_t track, map;

//    pthread_attr_t attr;
//    pthread_attr_init(&attr);

    pthread_create(&track, NULL, &tracking, args);
    pthread_create(&map, NULL, &mapping, args);

#if WITH_3DREP
    window.waitForKey();
    cnt = false;
#endif

    pthread_join(track, NULL);
    pthread_cancel(map);

#if WITH_3DREP
    window.addTextMessage(5, 5, mrpt::format("Press any key to exit"));
    window.repaint();

    window.waitForKey();
#endif

//    std::cout << "Loading..." << std::endl;

//    scene = window.get3DSceneAndLock();
//    scene->clear();
//    traj->clear();

//    scene->insert(traj);

//    mrpt::opengl::CPointCloudColouredPtr gl_points = mrpt::opengl::CPointCloudColoured::Create();
//    gl_points->enablePointSmooth(true);
//    gl_points->setPointSize(2);
//    scene->insert(gl_points);

//    mrpt::poses::CPose3D cam_pose(mrpt::poses::UNINITIALIZED_POSE);
//    mrpt::poses::CPose3D last_pose(mrpt::poses::UNINITIALIZED_POSE);
//    last_pose.setFromValues(0.0, 0.0, 0.0);

//    cam_pose = toCPose3D(opt[0]);

//    v_o[0]->setSensorPose(cam_pose);
//    v_o[0]->project3DPointsFromDepthImageInto(*gl_points, true);

//    traj->appendLine(last_pose.x(), last_pose.y(), last_pose.z(), cam_pose.x(), cam_pose.y(), cam_pose.z());
//    last_pose = cam_pose;

//    for (int k = 1; k < opt.size(); ++k)
//    {
//        gl_points = mrpt::opengl::CPointCloudColoured::Create();
//        gl_points->enablePointSmooth(true);
//        gl_points->setPointSize(2);
//        scene->insert(gl_points);

//        cam_pose = toCPose3D(opt[k]);

//        v_o[k]->setSensorPose(cam_pose);
//        v_o[k]->project3DPointsFromDepthImageInto(*gl_points, true);

//        traj->appendLine(last_pose.x(), last_pose.y(), last_pose.z(), cam_pose.x(), cam_pose.y(), cam_pose.z());
//        last_pose = cam_pose;
//    }

//    window.unlockAccess3DScene();
//    window.addTextMessage(5, 5, mrpt::format("Press any key to exit"));
//    window.repaint();

//    window.waitForKey();

    delete args;

    exit(0);
}


void *tracking(void *arg)
{
    //Tracking Thread
    args_t *args = (args_t*) arg;

    Difodo odo(args->downsample, args->ctf_levels);

    mrpt::poses::CPose3D last_pose(mrpt::poses::UNINITIALIZED_POSE);
    last_pose.setFromValues(0.0, 0.0, 0.0);

    int i = 0;

    mrpt::obs::CObservation3DRangeScanPtr obs = mrpt::obs::CObservation3DRangeScan::Create();
    while (cnt && args->input->read(obs))
    {
        //Processing...
        odo.compute(obs);



        cv::Mat rgb_image(obs->intensityImage.getAs<IplImage>());

        cv::Mat img_gray;
        cv::cvtColor(rgb_image, img_gray, CV_BGR2GRAY);

        std::vector<cv::KeyPoint> gray_k;
        std::vector<DBoW2::FBrief::TDescriptor> gray_d;

        cv::FAST(img_gray, gray_k, args->th, true);
        filter(gray_k, args->max_k);

        args->brief->compute(img_gray, gray_k, gray_d, true);

        bool is_keyframe = args->keyframe->process(gray_d);
        if (is_keyframe)
        {// TODO: accumulate cov
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            projectTo(cloud, obs, gray_k);

            Shared shared(gray_d, cloud, odo.cam_pose);

            pthread_mutex_lock(args->sync);
            //--------------------
            args->q->push(shared);
            args->v_o->push_back(obs);
            //--------------------
            pthread_mutex_unlock(args->sync);

            obs = mrpt::obs::CObservation3DRangeScan::Create();
            last_pose = odo.cam_pose;

            pthread_cond_signal(args->cond);
        }

#if !WITH_DATASET
        mrpt::system::sleep(33*args->skip); //TODO: clock_nanosleep
#endif

        obs->unload();
    }

    pthread_exit(NULL);
}

void *mapping(void *arg)
{
    //Mapping Thread
    args_t *args = (args_t*) arg;

    GraphOptimizer graph(false);

    mrpt::poses::CPose3D cam_pose(mrpt::poses::UNINITIALIZED_POSE);
    mrpt::poses::CPose3D last_pose(mrpt::poses::UNINITIALIZED_POSE);
    last_pose.setFromValues(0.0, 0.0, 0.0);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    std::vector<DBoW2::FBrief::TDescriptor> gray_d;

#if WITH_3DREP
    mrpt::opengl::CSetOfLinesPtr traj = mrpt::opengl::CSetOfLines::Create();
    traj->setLocation(0,0,0);
    traj->setColor(0,0.6,0);
    traj->setLineWidth(6);

    mrpt::opengl::COpenGLScenePtr scene;
    scene = args->window->get3DSceneAndLock();
    scene->insert(traj);
    args->window->unlockAccess3DScene();
#endif

    int k = 0;

    int last;
    while (cnt)
    {
        pthread_mutex_lock(args->sync);
        while (args->q->empty())
            pthread_cond_wait(args->cond, args->sync);

        Shared shared = args->q->front();
        args->q->pop();
        pthread_mutex_unlock(args->sync);

        gray_d = shared.brief;
        cloud = shared.cloud;
        cam_pose = shared.pose;

        int current = graph.addVertex(fromCPose3D(cam_pose));

        if (current > 0)
        {//Add odometry constraint (edge last -> current)
            //Pose increment (or edge):
            mrpt::poses::CPose3D inc(mrpt::poses::UNINITIALIZED_POSE);
            inc.inverseComposeFrom(cam_pose, last_pose);

            graph.addEdge(last, current, fromCPose3D(inc), Eigen::Matrix6d::Identity());

            // Trajectory
//            traj->appendLine(last_pose.x(), last_pose.y(), last_pose.z(), cam_pose.x(), cam_pose.y(), cam_pose.z());
        }

        //View
#if WITH_3DREP
        scene = args->window->get3DSceneAndLock();

        mrpt::opengl::CPointCloudColouredPtr gl_points = mrpt::opengl::CPointCloudColoured::Create();
        gl_points->enablePointSmooth(true);
        gl_points->setPointSize(2);
        scene->insert(gl_points);

        //TODO: or use at...
        args->v_o->operator [](current)->setSensorPose(cam_pose);
        args->v_o->operator [](current)->project3DPointsFromDepthImageInto(*gl_points, true);

        args->window->unlockAccess3DScene();
        args->window->repaint();
#endif

        last = current;

        DetectionResult result;
        args->detector->detectLoop(cloud, gray_d, result);

        if (result.detection())
        {
            std::cout << "Loop closure detected!!!" << std::endl;

            //Add loop closure constraint
            g2o::Isometry3D p;
            graph.getPose(result.match, p);

            g2o::Isometry3D t(result.transform.cast<double>());

            //Update pose
            p = p*t;
            graph.addLoop(result.match, result.query, t, p);

            std::cout << "Optimizing..." << std::endl;

            //Perform optimization
            graph.optimizeGraph();
        }
        else
            std::cout << k++ << "(no loop detected)" << std::endl;

        last_pose = cam_pose;
    }

    //graph.getPoses(opt);

    pthread_exit(NULL);
}
