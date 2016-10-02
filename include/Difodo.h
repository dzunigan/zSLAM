/**
* This file is part of zSLAM.
*
* Copyright (c) 2016 D. Zúñiga Noël
* For more information see <https://github.com/Peski/zSLAM>
*/

#ifndef Z_DIFODO_H
#define Z_DIFODO_H

//MRPT
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/vision/CDifodo.h>

class Difodo : public mrpt::vision::CDifodo {
public:

    /** Constructor. */
    Difodo(unsigned int downsample = 2, unsigned int ctf_levels = 5);

    /** Load the depth image and the corresponding groundtruth pose (backwards compatible)*/
    void loadFrame();
    void loadFrame(const mrpt::obs::CObservation3DRangeScanPtr &obs);

    /** Initialize the the visual odometry method (backwards compatible)*/
    void initialize();

    /** Odometry calculation or reset, when needed (backwards compatible)*/
    void compute();

    /** Odometry calculation or reset, when needed (backwards compatible)*/
    void compute(const mrpt::obs::CObservation3DRangeScanPtr &obs);

    /** Reset the class, calling initialize() method */
    void reset();

private:

    bool m_initialized;

    unsigned int m_downsample;

    unsigned int m_ctf_levels;

};

#endif
