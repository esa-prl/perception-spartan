#ifndef _CALIB_CONFIG_HPP_
#define _CALIB_CONFIG_HPP_

/**
 * This file contains the definitions of the various Config
 * objects that are used in this project. The actual values
 * that can are used when the project is run can be found
 * in the orogen/config folder and each .yml file there
 * corresponds to one or more items defined here. The values
 * seen there are automatically loaded and applied by modifying
 * the .rb script to apply the configuration.
 *
 * To automatically extract .yml files like these, use the
 * oroconf extract command from the terminal, used as defined
 * in the ROCK docs here:
 *
 * https://www.rock-robotics.org/documentation/tutorials/120_basics_configure_component.html
 * https://www.rock-robotics.org/documentation/runtime/configuration.html
 *
 * */

// WARNING: Whenever there is a "path" or "dir" parameter present, pass in
// a valid system path which MUST END IN A '/' character! This is important
// for when the path is concatenated with extensions for subdirectory/file
// access if you forget this file IO operations will almost certainly fail.


/*--- CALIBRATION FILE FORMAT (.yaml) ---*/
#define CALIB_IN_IMG_WIDTH "image_width"
#define CALIB_IN_IMG_HEIGHT "image_height"
#define CALIB_IN_CAMERA_MAT_LEFT "camera_matrix_1"
#define CALIB_IN_CAMERA_MAT_RIGHT "camera_matrix_2"
#define CALIB_IN_DIST_COEFFS_LEFT "distortion_coefficients_1"
#define CALIB_IN_DIST_COEFFS_RIGHT "distortion_coefficients_2"
#define CALIB_IN_ROT_MAT "rotation_matrix"
#define CALIB_IN_TRANS_COEFFS "translation_coefficients"

// Note: The above calibration format is assumed for compliance
// with the semantics of ImageLoader, however since you have access
// to the macros here, you may wish to rename things according to your
// specific needs. Just make sure that you understand what each yaml
// field should contain and remember that you need all of them present
// for this to work properly with the underlying OpenCV.

#define MAX_STR_LEN 256

namespace spartan
{

    struct OdometryConfig
    {
        int spartanWidth, spartanHeight;
        double vertical, horizontal, shiftBot;
        char calib_path[MAX_STR_LEN], dump_file_path[MAX_STR_LEN];
        OdometryConfig() :
            // Size of images fed into VO
            spartanWidth(512),
            spartanHeight(384),

            // These params are for cropping, don't touch them
            // unless you understand the semantics - see OdometryExecutor.cpp
            // By default they don't result in any cropping at all
            vertical(1.0),
            horizontal(1.0),
            shiftBot(0.0),

            // System params
            calib_path(""),
            dump_file_path("")
        {
        }
    };

    struct CameraEmuConfig
    {
        float delay;
        double rotx, roty, rotz, trax, tray, traz;
        int start_index;
        char img_dir[MAX_STR_LEN],
            right_dir[MAX_STR_LEN],
            left_dir[MAX_STR_LEN],
            left_mark[MAX_STR_LEN],
            right_mark[MAX_STR_LEN],
            img_file_format[MAX_STR_LEN];
        CameraEmuConfig() :
            // Delay in seconds between sending left and right frames,
            // positive means left comes first, negative means
            // right comes first.
            delay(0.01),

            // Sample transform data, change these to different values
            // if you need different transform outputs, or restructure
            // the whole thing if it suits your needs
            rotx(0.0),
            roty(0.0),
            rotz(0.0),
            trax(0.0),
            tray(0.0),
            traz(0.0),

            // Which index is the first to be read, use it to ignore
            // the first frames of your stream in case they are noisy
            // or boring.
            start_index(0),

            // System params
            img_dir(""),
            right_dir(""),
            left_dir(""),
            left_mark(""),
            right_mark(""),
            img_file_format("")
        {
        }
    };

    struct MappingConfig
    {
        int ndepths, matchKernelSide, mapWidth, mapHeight, start_frame, depthMode, sniper1MinNeighbors, sniper2MinNeighbors, radNN;
        double dstart, dend, verticalDistance, horizontalDistance,
               gridStep, xOffset, yOffset, correlationThresh,
               vertical, horizontal, shiftBot, voxelSize,
               voxelFactor, sniper1GrowthFactor, sniper1MinRadius,
               sniper1MaxRadius, sniper2GrowthFactor,
               sniper2MinRadius, sniper2MaxRadius, radRadius, truncNorm;
        bool includeColors;
        char calib_path[MAX_STR_LEN], dump_file_path[MAX_STR_LEN];
        MappingConfig() :
            // Size of images fed into the mapper
            mapWidth(1120),
            mapHeight(1120),

            // These params are for cropping, don't touch them
            // unless you understand the semantics - see OdometryExecutor.cpp
            // By default they don't result in any cropping at all
            vertical(1.0),
            horizontal(1.0),
            shiftBot(0.0),

            // Algorithmic params - see:
            // <SPARTAN_INSTALL_DIR>/software/vomap/src/sweep/{testSweep.c, spaceSweep.c}
            // for an entry point into how they are used internally.
            ndepths(101),
            depthMode(1),   //1=EXPONENTIAL_DEPTHS, 0=LINEAR_DEPTHS -> spaceSweep.h
            matchKernelSide(15),
            dstart(0.8),
            dend(4.00),
            verticalDistance(0.77),
            horizontalDistance(0.77),
            gridStep(0.0007),
            xOffset(0.0),
            yOffset(0.0),
            correlationThresh(0.65),

            // PCL range limiting
            truncNorm(6.5),

            // PCL voxelization
            voxelFactor(1.0),   // <= 0.0 means cubes, anything else means rectangles with square base
            voxelSize(0.05),    // the height of which is voxelSize x voxelSize x (voxelSize*voxelFactor)

            // Denoising
            sniper1MinNeighbors(5),
            sniper1GrowthFactor(5.5),
            sniper1MinRadius(0.1),
            sniper1MaxRadius(0.3),
            sniper2MinNeighbors(4),
            sniper2GrowthFactor(6),
            sniper2MinRadius(0.05),
            sniper2MaxRadius(0.15),
            radNN(4),
            radRadius(0.05),

            // System params
            start_frame(0),
            includeColors(true),
            calib_path(""),
            dump_file_path("")
        {
        }
    };
}



#endif  // _CALIB_CONFIG_HPP_
