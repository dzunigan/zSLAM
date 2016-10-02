/**
* This file is part of zSLAM.
*
* Copyright (c) 2016 D. Zúñiga Noël
* For more information see <https://github.com/Peski/zSLAM>
*/

//MRPT
#include <mrpt/hwdrivers/COpenNI2Sensor.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/system/threads.h>
#include <mrpt/synch/CSemaphore.h>

//For 3D dense representation
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/CPointCloudColoured.h>

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
#include <thread>

//OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "TemplatedLoopDetector3D.h"
#include "TemplatedKeyframeSelection.h"
#include "Dataset.h"
#include "Difodo.h"
#include "GraphOptimizer.h"

#include "util.h"

/// BRIEF-based
typedef TemplatedLoopDetector3D
  <DBoW2::FBrief::TDescriptor, DBoW2::FBrief> BriefLoopDetector;
typedef TemplatedKeyframeSelection
  <DBoW2::FBrief::TDescriptor, DBoW2::FBrief> BriefKeyframeSelection;

int main(int argc, char** argv)
{
    std::cout << "Copyright (c) 2016, D. Zúñiga Noël" << std::endl;
    std::cout << std::endl;

//    wait();

    std::string config_path = "./data/config";

    if (argc > 1)
        config_path = argv[1];

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

    // Load the vocabulary to use
    std::cout << "Loading vocabulary..." << std::endl;
    BriefVocabulary voc(vocabulary);

    cv::String pattern_file(pattern);
    cv::FileStorage fs(pattern_file, cv::FileStorage::READ);
    if(!fs.isOpened()) throw std::string("Could not open pattern file");

    std::vector<int> x1, y1, x2, y2;
    fs["x1"] >> x1;
    fs["x2"] >> x2;
    fs["y1"] >> y1;
    fs["y2"] >> y2;

    DVision::BRIEF brief;
    brief.importPairs(x1, y1, x2, y2);

    //For 3D dense reconstruction
    mrpt::gui::CDisplayWindow3D window;
    mrpt::opengl::COpenGLScenePtr scene;
    window.setWindowTitle("Map");
    window.resize(800,600);
    window.setPos(500,50);
    window.setCameraZoom(16);
    window.setCameraAzimuthDeg(0);
    window.setCameraElevationDeg(45);

    // Open sensor:
    // ----------------------------------
    mrpt::hwdrivers::COpenNI2Sensor rgbd_sensor;
    unsigned sensor_id_or_serial = 0;
    rgbd_sensor.initialize();

    if(rgbd_sensor.getNumDevices() == 0)
        return 0;

    std::cout << rgbd_sensor.getNumDevices() << " devices available."  << std::endl;
    std::cout << "Using device " << sensor_id_or_serial << std::endl;

    //Thread synch
    mrpt::synch::CSemaphore sync(1, 1, "track-map");

    //Shared data between threads
    std::queue< std::vector<DBoW2::FBrief::TDescriptor> > q_d;
    std::queue<pcl::PointCloud<pcl::PointXYZ>::Ptr> q_c;
    std::queue<mrpt::poses::CPose3D> q_o;

    std::vector<mrpt::obs::CObservation3DRangeScanPtr> v_o; //For 3D dense reconstruction

    //Final trajectory
    std::vector<g2o::Isometry3D> opt;

    //User-fired termination
    bool finish = false;
    mrpt::synch::CSemaphore s_end(1, 1, "finish");

    window.waitForKey();

    std::cout << "Running..." << std::endl;

    #pragma omp parallel sections
    {
        #pragma omp section
        {//Tracking Thread
            mrpt::system::sleep(1000);

            Difodo odo(downsample, ctf_levels);
            BriefKeyframeSelection keyframe(voc, beta);

            mrpt::poses::CPose3D last_pose(mrpt::poses::UNINITIALIZED_POSE);
            last_pose.setFromValues(0.0, 0.0, 0.0);

            mrpt::obs::CObservation3DRangeScanPtr obs = mrpt::obs::CObservation3DRangeScan::Create();
            bool bObs = false, bError = true;
            rgbd_sensor.getNextObservation(*obs, bObs, bError);
            bool end = (!bObs || bError);
            while (!end)
            {
                //Processing...
                odo.compute(obs);

                cv::Mat rgb_image(obs->intensityImage.getAs<IplImage>());

                cv::Mat img_gray;
                cv::cvtColor(rgb_image, img_gray, CV_BGR2GRAY);

                std::vector<cv::KeyPoint> gray_k;
                std::vector<DBoW2::FBrief::TDescriptor> gray_d;

                cv::FAST(img_gray, gray_k, th, true);
                filter(gray_k, max_k);

                brief.compute(img_gray, gray_k, gray_d, true);

                bool is_keyframe = keyframe.process(gray_d);
                if (is_keyframe)
                {
                    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
                    projectTo(cloud, obs, gray_k);

                    sync.waitForSignal();
                    //--------------------
                    q_d.push(gray_d);
                    q_c.push(cloud);
                    q_o.push(odo.cam_pose);
                    v_o.push_back(obs);
                    //--------------------
                    sync.release();

                    obs = mrpt::obs::CObservation3DRangeScan::Create();
                    last_pose = odo.cam_pose;
                }

                mrpt::system::sleep(33*skip);

                rgbd_sensor.getNextObservation(*obs, bObs, bError);
                end = (!bObs || bError);

                s_end.waitForSignal();
                //--------------------
                end |= finish;
                finish = end;
                //--------------------
                s_end.release();
            }
        }

        #pragma omp section
        {//Mapping Thread
            mrpt::system::sleep(1000);

            BriefLoopDetector::Parameters params(   480,   640,    f, true, alpha, 0,                        GEOM_DI,           4);

            // Initiate loop detector with the vocabulary
            BriefLoopDetector detector(voc, params);

            mrpt::poses::CPose3D cam_pose(mrpt::poses::UNINITIALIZED_POSE);
            mrpt::poses::CPose3D last_pose(mrpt::poses::UNINITIALIZED_POSE);
            last_pose.setFromValues(0.0, 0.0, 0.0);

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
            std::vector<DBoW2::FBrief::TDescriptor> gray_d;

            GraphOptimizer g(false);
            int last;

            bool end = false;
            while (!end)
            {
                int n;
                sync.waitForSignal();
                assert((q_o.size() == q_c.size()) &&
                       (q_c.size() == q_d.size()));

                n = q_o.size();
                if (n > 0)
                {
                    cam_pose = q_o.front();
                    q_o.pop();

                    cloud = q_c.front();
                    q_c.pop();

                    gray_d = q_d.front();
                    q_d.pop();
                }
                sync.release();

                if (n > 0)
                {
                    int current = g.addVertex(fromCPose3D(cam_pose));

                    if (current > 0)
                    {//Add odometry constraint (edge last -> current)
                        //Pose increment (or edge):
                        mrpt::poses::CPose3D inc(mrpt::poses::UNINITIALIZED_POSE);
                        inc.inverseComposeFrom(cam_pose, last_pose);

                        g.addEdge(last, current, fromCPose3D(inc), Eigen::Matrix6d::Identity());
                    }

                    //View
                    scene = window.get3DSceneAndLock();

                    mrpt::opengl::CPointCloudColouredPtr gl_points = mrpt::opengl::CPointCloudColoured::Create();
                    gl_points->enablePointSmooth(true);
                    gl_points->setPointSize(2);
                    scene->insert(gl_points);

                    v_o[current]->setSensorPose(cam_pose);
                    v_o[current]->project3DPointsFromDepthImageInto(*gl_points, true);

                    window.unlockAccess3DScene();
                    window.repaint();
                    //--

                    last = current;

                    DetectionResult result;
                    detector.detectLoop(cloud, gray_d, result);

                    if (result.detection())
                    {
                        std::cout << "Loop closure detected!!!" << std::endl;

                        //Add loop closure constraint
                        g2o::Isometry3D p;
                        g.getPose(result.match, p);

                        g2o::Isometry3D t(result.transform.cast<double>());

                        //Update pose
                        p = p*t;
                        g.addLoop(result.match, result.query, t, p);

                        //Perform optimization
                        g.optimizeGraph();

                        std::cout << "Press enter to continue..." << std::endl;

                        end = true;
                    }
                    else
                        std::cout << "No loop detected..." << std::endl;

                    last_pose = cam_pose;
                }
                else
                {
                    mrpt::system::sleep(17);
                }

                s_end.waitForSignal();
                //--------------------
                end |= finish;
                finish = end;
                //--------------------
                s_end.release();
            }

            g.getPoses(opt);
        }
//        #pragma omp section
//        {//User-fired termination
//            std::cout << "Press enter to stop" << std::endl;
//            std::getchar();

//            s_end.waitForSignal();
//            //--------------------
//            finish = true;
//            //--------------------
//            s_end.release();
//        }
    }

    window.waitForKey();

    scene = window.get3DSceneAndLock();
    scene->clear();
    for (int k = 0; k < opt.size(); ++k)
    {
        mrpt::opengl::CPointCloudColouredPtr gl_points = mrpt::opengl::CPointCloudColoured::Create();
        gl_points->enablePointSmooth(true);
        gl_points->setPointSize(2);
        scene->insert(gl_points);

        v_o[k]->setSensorPose(toCPose3D(opt[k]));
        v_o[k]->project3DPointsFromDepthImageInto(*gl_points, true);
    }

    window.unlockAccess3DScene();
    //window.addTextMessage(5, 5, mrpt::format("Press any key to exit"));
    window.repaint();

    window.waitForKey();

    return 0;
}
