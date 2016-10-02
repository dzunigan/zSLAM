/**
* This file is part of zSLAM.
*
* Copyright (c) 2016 D. Zúñiga Noël
* For more information see <https://github.com/Peski/zSLAM>
*/

#include "Dataset.h"

//STL
#include <stdexcept>

typedef mrpt::obs::CObservation3DRangeScan CObservation3DRangeScan;

void Dataset::loadDataset(const std::string &filename)
{
    if (!dataset.loadFromRawLogFile(filename))
        throw std::runtime_error("Couldn't open rawlog dataset file for input...");

    // Set external images directory:
    const std::string imgsPath = mrpt::obs::CRawlog::detectImagesDirectory(filename);
    mrpt::utils::CImage::IMAGES_PATH_BASE = imgsPath;

    rawlog_count = 0;
}

bool Dataset::read(mrpt::obs::CObservation3DRangeScanPtr &obs)
{
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
    rawlog_count++;
}
