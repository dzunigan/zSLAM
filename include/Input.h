/**
* This file is part of zSLAM.
*
* Copyright (c) 2016-2017 D. Zúñiga Noël
* For more information see <https://github.com/Peski/zSLAM>
*/

#ifndef Z_INPUT_H
#define Z_INPUT_H

//MRPT
#include <mrpt/obs/CRawlog.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/hwdrivers/COpenNI2Sensor.h>

//STL
#include <string>

#include "config.h"

class Input
{
public:

#if WITH_DATASET
    void loadDataset(const std::string &filename);
#else
    void openSensor(const unsigned serial);
#endif

    bool read(mrpt::obs::CObservation3DRangeScanPtr &obs);

protected:

#if WITH_DATASET
    mrpt::obs::CRawlog dataset;
    unsigned int rawlog_count;
#else
    mrpt::hwdrivers::COpenNI2Sensor rgbd_sensor;
#endif

};

#endif
