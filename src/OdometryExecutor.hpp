#ifndef _ODOMETRY_EXECUTOR_HPP_
#define _ODOMETRY_EXECUTOR_HPP_

#include <spartan/Config.hpp>
#include <spartan/CalibInfo.hpp>
#include <spartan/ImageLoader.hpp>
#include <base/Time.hpp>
#include <base/samples/RigidBodyState.hpp>

#include <climits>
#include <tuple>
#include <iostream>
#include <vector>
#include <cstring>
#include <cmath>
#include<fstream>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

/* SPARTAN */
#include <vo.h>
#include <sam.h>
#include <pipeline.h>
#include <gettimeofday.h>
#include <undistort.h>
/***********/

#define NUM_QRTPARAMS 7
#define NUM_PPARAMS 12
#define NUM_RTPARAMS 6

#define MAXSTRLEN 512
#define THE_FRAME_STEP 1
#define VO_MAXMAXCORNERS 3000
#define NUM_DISTORT 5
#define DISTORT_READ_THRESH 1e-6

using namespace spartan;
using namespace Eigen;

namespace spartan
{

class OdometryExecutor
{

    public:
        OdometryExecutor(const OdometryConfig&, CalibInfo, Affine3d);
        ~OdometryExecutor();
        void nextPose(base::samples::frame::FramePair, std::vector<double>&);
        base::samples::RigidBodyState getRBS();
        vostate *getVisOdomState();
        base::samples::frame::Frame *getImgFrame(int);
    private:
        ///// ROCK INTEGRATION UTILITIES  /////
        uint32_t last_pair_uid;
        bool vo_initialized;
        void initializeVisOdomAgent(uint16_t, uint16_t);
        Eigen::Affine3d base_tf, inv_base_tf, sec_round_offset, inv_sec_round_offset;
        bool undistortParamsPresent();
        void cropTopAndLR(uint8_t *, int, int, bool, int, int, int, uint8_t*);
        std::vector<base::samples::RigidBodyState> rbsRegistry;
        OdometryConfig mConfig;

        // Devon utils
        bool devon;
        void dumpPoseToFile(double[], char*, bool);

        ///// SPARTAN NATIVE BELOW /////
        /* driver */
        char fname[MAXSTRLEN];
        FILE *fp;
        int Ln, Rn;
        unsigned char *Limgdata=NULL, *Rimgdata=NULL;
        char Ltmpl[MAXSTRLEN], Rtmpl[MAXSTRLEN];
        int Lwidth, Lheight, Rwidth, Rheight;
        int bpp, isjpg, frstep=THE_FRAME_STEP, nmaxframes=INT_MAX;
        float cycle_start, cycle_end, start_time, end_time;
        double rt[NUM_RTPARAMS], rtcovar[NUM_RTPARAMS*NUM_RTPARAMS];
        double qrt[NUM_QRTPARAMS];
        double distort_coeffs_left[NUM_DISTORT], distort_coeffs_right[NUM_DISTORT];
        double pts3D[VO_MAXMAXCORNERS][3], pts3Dcovar[VO_MAXMAXCORNERS][9];
        int npts3D;
        struct vostate *vos;

        double LPmat[NUM_PPARAMS], RPmat[NUM_PPARAMS], r[3], t[3];
        double campose0[NUM_RTPARAMS];

        //double K[9]={ // CHECKME
        // 394.2054307, 0, 256,
        // 0, 394.2054307, 192,
        // 0, 0, 1
        //};

        double K_LEFT[9], K_RIGHT[9], KLC[9], KRC[9];

        //double K[9] = {
        //    834, 0, 511.5,
        //    0, 834, 378.5,
        //    0, 0, 1
        //};

        double baseline;
        /////////////////////////////////////////////////////
        struct timeval start, stop;
        double accum;
        /////////////////////////////////////////////////////

        char gtfname[MAXSTRLEN];
        double trajlen=0.0, *gtPoses=NULL;

}; // end OdometryExecutor


} // end namespace spartan

#endif // _ODOMETRY_EXECUTOR_HPP_
