/**
* This file is part of zSLAM.
*
* Copyright (c) 2016-2017 D. Zúñiga Noël
* For more information see <https://github.com/Peski/zSLAM>
*/

#include "Input.h"

//STL
#include <stdexcept>

typedef mrpt::obs::CObservation3DRangeScan CObservation3DRangeScan;

#if WITH_DATASET
void Input::loadDataset(const std::string &filename)
{
    if (!dataset.loadFromRawLogFile(filename))
        throw std::runtime_error("Couldn't open rawlog dataset file for input...");

    // Set external images directory:
    const std::string imgsPath = mrpt::obs::CRawlog::detectImagesDirectory(filename);
    mrpt::utils::CImage::IMAGES_PATH_BASE = imgsPath;

    rawlog_count = 0;
}
#else
void Input::openSensor(const unsigned serial)
{
    rgbd_sensor.setSerialToOpen(serial);

    rgbd_sensor.initialize();

    if(rgbd_sensor.getNumDevices() == 0)
        throw std::runtime_error("Couldn't open rgbd sensor device for input...");
}
#endif

bool Input::read(mrpt::obs::CObservation3DRangeScanPtr &obs)
{
#if WITH_DATASET
    mrpt::obs::CObservationPtr alfa;

    if (dataset.size() <= rawlog_count)
        return false;

    alfa = dataset.getAsObservation(rawlog_count);

    while (!IS_CLASS(alfa, CObservation3DRangeScan))
    {
        rawlog_count++;
        if (dataset.size() <= rawlog_count)
            return false;

        alfa = dataset.getAsObservation(rawlog_count);
    }

    obs = mrpt::obs::CObservation3DRangeScanPtr(alfa);
    obs->load();
    rawlog_count++;

    return true;
#else
    bool bObs = false, bError = true;

    rgbd_sensor.getNextObservation(*obs, bObs, bError);

    return (bObs && !bError);
#endif
}
