/**
* This file is part of zSLAM.
*
* Copyright (c) 2016 D. Zúñiga Noël
* For more information see <https://github.com/Peski/zSLAM>
*/

#ifndef Z_DATASET_H
#define Z_DATASET_H

//MRPT
#include <mrpt/obs/CRawlog.h>
#include <mrpt/obs/CObservation3DRangeScan.h>

//STL
#include <string>

class Dataset
{
public:

    void loadDataset(const std::string &filename);

    bool read(mrpt::obs::CObservation3DRangeScanPtr &obs);

protected:

    mrpt::obs::CRawlog dataset;
    std::ifstream f_gt;

    unsigned int rawlog_count;
};

#endif
