#ifndef _MAPPING_EXECUTOR_HPP_
#define _MAPPING_EXECUTOR_HPP_

#include <spartan/Config.hpp>
#include <spartan/CalibInfo.hpp>
#include <spartan/ImageLoader.hpp>
#include <base/Time.hpp>
#include <spartan/SpartanMap.hpp>

#include <climits>
#include <iostream>
#include <vector>
#include <cstring>
#include <cmath>
#include <fstream>

#include <Eigen/Dense>

/* SPARTAN */
extern "C" {
#include <typesnconsts.h>
#include <matrixOps.h>
#include <debugfileIO.h>
#include <sweepFileIO.h>
#include <vgg_interp2.h>
#include <spaceSweep.h>
#include <sweep/geometry.h>
#include <sweep/cameraGeometry.h>
}
#include <lodepng.h>
#include <undistort.h>
#include <sam.h>
/***********/

#define NUM_QRTPARAMS 7
#define NUM_PPARAMS 12
#define NUM_RTPARAMS 6

#define MAXSTRLEN 512
#define THE_FRAME_STEP 1
#define VO_MAXMAXCORNERS 3000
#define NUM_DISTORT 5
#define DISTORT_READ_THRESH 1e-6


//#define DEBUG_FILE_NO_TF "/home/marta/compass/software/vomap/src/build/sweep/outres/original_map.txt"

using namespace std;
using namespace spartan;
using namespace Eigen;

namespace spartan
{

class MappingExecutor
{

    public:
        MappingExecutor(const MappingConfig&, CalibInfo, Affine3d);
        ~MappingExecutor();

        void generateMap(base::samples::frame::FramePair, Affine3d);
        base::samples::frame::Frame *getImgFrame(int);
        base::samples::Pointcloud getTransportable();
    private:
        /* ROCK INTEGRATION */
        MappingConfig mConfig;
        SpartanMap *mpSpartanMap;
        int mCurrentFrame, Lheight, Lwidth, Rheight, Rwidth;
        Affine3d base_tf, inv_base_tf;

        void initializeSPSParams();
        bool undistortParamsPresent();
        void cropTopAndLR(uint8_t*, int, int, bool, int, int ,int ,uint8_t*);

        uint8_t *Limgdata, *Rimgdata;

        /* CALIB VARS - FOR FORWARD COMPAT */
        double K_LEFT[9], K_RIGHT[9], KLC[9], KRC[9],
            distort_coeffs_left[5], distort_coeffs_right[5],
            r[3], t[3];


        /* SPARTAN NATIVE */
        double P0[NUM_PPARAMS], P1[NUM_PPARAMS];
        struct SweepParams *sp_params;


}; // end MappingExecutor


} // end namespace spartan

#endif // _MAPPING_EXECUTOR_HPP_
