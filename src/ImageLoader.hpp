#ifndef _SPARTAN_IMAGE_LOADER_HPP_
#define _SPARTAN_IMAGE_LOADER_HPP_

#include <base/samples/Frame.hpp>
#include <spartan/Config.hpp>
#include <spartan/CalibInfo.hpp>

#include <opencv2/opencv.hpp>

#include <vector>
#include <stdlib.h>
#include <iostream>
#include <string>

#define LEFT_CAMERA_FEED 0
#define RIGHT_CAMERA_FEED 1

// Utility
#define MAX_STR_LEN 256


using namespace base::samples::frame;
using namespace spartan;

namespace spartan
{
    class ImageLoader
    {
        protected:
            Frame *getGrayscaleEquiv(Frame&);
            std::vector<uint8_t> grayscalify(std::vector<uint8_t>);
            bool timestampThreshCheck();
            bool properPairStored();
            void applyOpticalCorrections(Frame&, Frame&);
            void getFinalMatrices();
            cv::Size getIntermediateSize();
            void exportUpdatedCalibrationParams(cv::Mat, double*, double*, double*, double*, double*, double*);

            int orig_width, orig_height, spartan_width, spartan_height;
            char calib_full_path[MAX_STR_LEN];
            bool left_fresh, right_fresh;
            uint32_t lfc, rfc;
            uint32_t uid_counter;
            Frame *frame_left;
            Frame *frame_right;
            cv::Mat lm1, lm2, rm1, rm2, p1, p2;
            cv::Size intermediate_sz;

        public:
            ImageLoader(const OdometryConfig&);
            ~ImageLoader();
            void newFrame(int, Frame*);
            FramePair *getFramePair();
            CalibInfo getCalibInfo();

    };  // end class ImageLoader

} // end namespace spartan

#endif // _SPARTAN_IMAGE_LOADER_HPP_

