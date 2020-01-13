#include "MappingExecutor.hpp"

using namespace spartan;
using namespace std;
using namespace Eigen;

MappingExecutor::MappingExecutor(const MappingConfig& conf, CalibInfo ci, Affine3d cam2body_tf)
{
    mConfig = conf;

    mCurrentFrame = mConfig.start_frame;
    mpSpartanMap = NULL;
    Limgdata = NULL;
    Rimgdata = NULL;

    /* TRANSFORM INITIALIZATIONS */
    base_tf = cam2body_tf;
    inv_base_tf = base_tf.inverse();


    /* Projection matrix & distortion param loading */
    // CAREFUL: SPARTAN uses Rodriguez convention, but we have a full rotation
    // matrix from the calibration file, we need conversion logic, which luckily
    // can be dug out of the SAM libraries bundled with SPARTAN with a bit of
    // patience (see below)
    double rot_mat[9];

    memcpy(K_LEFT, ci.K_LEFT, 9*sizeof(ci.K_LEFT[0]));
    memcpy(KLC, ci.K_LEFT, 9*sizeof(ci.K_LEFT[0]));
    KLC[2] -= mConfig.mapWidth * (1.0-mConfig.horizontal);
    KLC[5] -= mConfig.mapHeight * (1.0-mConfig.vertical-mConfig.shiftBot);
    memcpy(distort_coeffs_left, ci.D_LEFT, NUM_DISTORT*sizeof(ci.D_LEFT[0]));

    // Copy over the rotation matrix (3x3 = 9 doubles)
    memcpy(rot_mat, ci.ROT_LEFT, 9*sizeof(ci.ROT_LEFT[0]));

    // Now call the SAM library function to do the convertion, see
    // <SPARTAN_INSTALL_DIR>/software/vomap/src/VO/sam.c for implementation
    sam_rotmat2vec(rot_mat, r);

    memcpy(t, ci.T_LEFT, 3*sizeof(ci.T_LEFT[0]));
    sam_PfromKrt(KLC, r, t, P0);

    // Do the same for the right camera
    memcpy(K_RIGHT, ci.K_RIGHT, 9*sizeof(ci.K_RIGHT[0]));
    memcpy(KRC, ci.K_RIGHT, 9*sizeof(ci.K_RIGHT[0]));
    KRC[5] -= mConfig.mapHeight * (1.0-mConfig.vertical-mConfig.shiftBot);
    memcpy(distort_coeffs_right, ci.D_RIGHT, NUM_DISTORT*sizeof(ci.D_RIGHT[0]));
    memcpy(rot_mat, ci.ROT_RIGHT, 9*sizeof(ci.ROT_RIGHT[0]));
    sam_rotmat2vec(rot_mat, r);
    memcpy(t, ci.T_RIGHT, 3*sizeof(ci.T_RIGHT[0]));
    sam_PfromKrt(KRC, r, t, P1);

    std::cout << "\n\nMATRIX DEBUG:\n"
        << KRC[0] << ", " << K_RIGHT[0] << "\n"
        << KRC[1] << ", " << K_RIGHT[1] << "\n"
        << KRC[2] << ", " << K_RIGHT[2] << "\n"
        << KRC[3] << ", " << K_RIGHT[3] << "\n"
        << KRC[4] << ", " << K_RIGHT[4] << "\n"
        << KRC[5] << ", " << K_RIGHT[5] << "\n"
        << KRC[6] << ", " << K_RIGHT[6] << "\n"
        << KRC[7] << ", " << K_RIGHT[7] << "\n"
        << KRC[8] << ", " << K_RIGHT[8] << "\n"
        << "\n\n\n";

    std::cout << "\nPROJECTIONS DEBUG:\n";
    std::cout << "LEFT\n";
    for (double x : P0)
        std::cout << x << ", ";
    std::cout << std::endl;
    std::cout << "RIGHT\n";
    for (double x : P1)
        std::cout << x << ", ";
    std::cout << std::endl;

    initializeSPSParams();
}


/**
 * This method initializes the space sweep parameter struct by wrapping
 * the corresponding SPARTAN library function and storing the result in the
 * appropriate member variable of this class.
 * */
void MappingExecutor::initializeSPSParams()
{
    std::cout << "Initializing SPS parameters... ";
    sp_params = sps_init(
            sp_params, NULL, mConfig.ndepths, mConfig.dstart, mConfig.dend,
            mConfig.depthMode, mConfig.gridStep, mConfig.horizontalDistance,
            mConfig.verticalDistance, mConfig.xOffset, mConfig.yOffset,
            P0, P1, mConfig.matchKernelSide, mConfig.correlationThresh
    );

    if (!sp_params) {
        std::cerr << "Failed to initialize Space Sweep parameters!" << std::endl;
        exit(1);
    }
    std::cout << "DONE" << std::endl;
}

MappingExecutor::~MappingExecutor()
{
    free(mpSpartanMap);
    free(Limgdata);
    free(Rimgdata);
    sps_cleanup(sp_params);
}


/**
 * This method is uded internally to check whether undistortion
 * parameters were read from the calibration files that are
 * larger than some very small threshold, meaning that bouguet
 * undistortion is necessary.
 * */
bool MappingExecutor::undistortParamsPresent()
{
    for (int i=0; i<NUM_DISTORT; i++) {
        if (fabs(distort_coeffs_left[i]) > DISTORT_READ_THRESH
                || fabs(distort_coeffs_right[i]) > DISTORT_READ_THRESH)
            return true;
    }
    return false;
}


/**
 * This function accepts an original image along with its dimensions,
 * as well as output dimensions and an output PREALLOCATED image buffer.
 * The result after cropping is that the image loses pixels from the
 * top rows equal to (orig_height - final_height) and shifts the selection
 * area upwards by shift_bot pixels. The area is also constricted horizontally
 * by (orig_width - final_width) pixels, subtracted from the left if isLeft=true,
 * from the right otherwise.
 *
 * xxxxxx
 * xxxxxx
 * xoooox
 * xoooox
 * xxxxxx
 *
 * Original image: height = 7 rows, width = 6 cols
 * Final image: height = 2 rows, width = 4 cols
 * Rejected:
 *  2 columns (1 on either side)
 *  3 rows (2 top and 1 bot)
 * */
void MappingExecutor::cropTopAndLR(uint8_t *orig_img, int orig_width, int orig_height, bool isLeft, int final_width, int final_height, int shift_bot, uint8_t *res)
{
    std::cout << "CROPPING" << std::endl;
    // Calculate the indices that bound the region we need to keep
    int topOffset = orig_height - final_height - shift_bot;
    int leftOffset = ((isLeft) ? orig_width - final_width : 0);

    // Copy the required area to the output array
    for (int i=topOffset; i<orig_height-shift_bot; i++) {
        memcpy(
                res+(i-topOffset)*final_width,  // final image row
                orig_img+i*orig_width+leftOffset,   // original image region
                final_width*sizeof(uint8_t) // (final image row size) number of pixels
        );
    }
    std::cout << "CROPPED" << std::endl;
    return;
}


/**
 * This method generates a point map and transforms it so that all points (which are internally
 * computed by SPARTAN with respect to the camera frame) are expressed in world coordinates. To
 * do this, first apply the base_tf to go from camera to body and then apply the worldBodyPose
 * transform to express the new points with respect to the world coordinate system. For efficiency
 * compose the transforms and then apply them in one sweep to the quite large map that is created
 * by SPARTAN.
 * */
void MappingExecutor::generateMap(base::samples::frame::FramePair stereo_pair, Affine3d world2body_tf)
{
    unsigned char *Limg_distorted, *Rimg_distorted;

    // Get the data, assumed to be grayscale by now
    Limg_distorted = stereo_pair.first.getImagePtr();
    Rimg_distorted = stereo_pair.second.getImagePtr();

    // Frames are implicitly assumed to be of equal size
    base::samples::frame::frame_size_t frame_sz = stereo_pair.first.getSize();
    Lwidth = frame_sz.width;
    Rwidth = Lwidth;
    Lheight = frame_sz.height;
    Rheight = Lheight;

    // In case distortion params are present, undistortion needs
    // to be applied. For more info, please refer to the SPARTAN
    // code at:
    // <SPARTAN_INSTALL_DIR>/software/vomap/src/multires/undistort.c
    if (Limgdata == NULL) {
        Limgdata = (unsigned char *) malloc(
                Lwidth * Lheight * sizeof(unsigned char));
    } else {
        Limgdata = (unsigned char *) realloc(Limgdata,
                Lwidth * Lheight * sizeof(unsigned char));
    }
    if (Rimgdata == NULL) {
        Rimgdata = (unsigned char *) malloc(
                Rwidth * Rheight * sizeof(unsigned char));
    } else {
        Rimgdata = (unsigned char *) realloc(Rimgdata,
                Rwidth * Rheight * sizeof(unsigned char));
    }

    if (undistortParamsPresent()) {
        std::cout << "UNDISTORTING" << std::endl;
        bouguet_imgundistort(
                Limg_distorted,
                Lwidth,
                Lheight,
                distort_coeffs_left,
                4,  // number of non-zero distort coeffs for BB2@exoter, edit according to your camera's specs
                K_LEFT,
                Limgdata
        );
        bouguet_imgundistort(
                Rimg_distorted,
                Rwidth,
                Rheight,
                distort_coeffs_right,
                4,  // same as for left
                K_RIGHT,
                Rimgdata
        );
    } else {
        // Else simply store the images directly, since no undistortion
        // is required. This should be the case if all distortion
        // coefficients are zero, to avoid unnecessary function calls,
        // though the result should not change.
        memcpy(Limgdata, Limg_distorted, Lwidth*Lheight*sizeof(unsigned char));
        memcpy(Rimgdata, Rimg_distorted, Rwidth*Rheight*sizeof(unsigned char));
    }


    /* DEBUGGING */
    /*cout << "DATA DEBUG:\n" << (int)Limgdata[0] << endl;
    cout << (int)Rimgdata[0] << endl;

    cout << "EXTENDED DEBUG: L("
        << Lwidth << ", "
        << Lheight << ") - R("
        << Rwidth << ", "
        << Rheight << ")"
        << endl;
    */

    int effective_height = (int) (Lheight * mConfig.vertical);
    int effective_width = (int) (Lwidth * mConfig.horizontal);
    int bottom_shift = (int) (Lheight * mConfig.shiftBot);

    /*--- IMAGE CROPPING ---*/
    unsigned char *LimgCropped = (uint8_t*)malloc(effective_width * effective_height * sizeof(uint8_t));
    unsigned char *RimgCropped = (uint8_t*)malloc(effective_width * effective_height * sizeof(uint8_t));
    cropTopAndLR(Limgdata, Lwidth, Lheight, true, effective_width, effective_height, bottom_shift, LimgCropped);
    cropTopAndLR(Rimgdata, Rwidth, Rheight, false, effective_width, effective_height, bottom_shift, RimgCropped);

    Limgdata = (uint8_t*) realloc(Limgdata, effective_width * effective_height * sizeof(uint8_t));
    Rimgdata = (uint8_t*) realloc(Rimgdata, effective_width * effective_height * sizeof(uint8_t));
    memcpy(Limgdata, LimgCropped, effective_width * effective_height * sizeof(uint8_t));
    memcpy(Rimgdata, RimgCropped, effective_width * effective_height * sizeof(uint8_t));
    free(LimgCropped);
    free(RimgCropped);
    /************************/

    /*
    std::cout
        << (int) Limgdata[0] << ", "
        << (int) Limgdata[100] << ", "
        << (int) Limgdata[200] << ", "
        << (int) Rimgdata[0] << ", "
        << (int) Rimgdata[100] << ", "
        << (int) Rimgdata[200] << endl;
    */


    // Call the SPARTAN mapper, this step may be running on an FPGA
    // depending on what HW configuration the SPARTAN library was
    // compiled with. In that case, params related to image sizing
    // may be hardcoded onto the FPGA and impossible to change
    // without altering the bitstream. To accomodate for this,
    // change the ImageLoader config file to get the images to be
    // the size you need them to be in.
    std::cout << "Executing Space Sweep...";
    std::cout << "ew " << effective_width << ", eh " << effective_height << std::endl;
    sps_spaceSweep(sp_params, Limgdata, Rimgdata, effective_width, effective_height);
    std::cout << "DONE" << std::endl;

    // Internally, sp_params now contains a pointer to the map we need.
    // This map is structured as 3D coordinates of points, stored in the
    // sp_params->ptsMax variable. However, these points are expressed
    // in the camera frame, which is not necessarily desirable. Therefore
    // we push the data to a custom class that can execute transforms using
    // Eigen, so that we may express the data directly in our world
    // frame. The class also allows dumping everything to a text file,
    // as well as grayscale sampling for "coloring" the map.
    std::cout << "Initializing internal map entity...";
    mpSpartanMap = new SpartanMap(sp_params, P0, Limgdata, effective_width, effective_height, mCurrentFrame);
    std::cout << "DONE" << std::endl;

    // Find the cumulative world to camera transform, by
    // concatenating world->body with body->left_camera,
    // then use the result to transform the map
    Affine3d total_tf;
    total_tf = world2body_tf * base_tf;

#ifdef DEBUG_FILE_NO_TF
    mpSpartanMap->printToFile(
            DEBUG_FILE_NO_TF,
            true,
            true
    );
#endif
    
    std::cout << "Voxelizing cloud...\n";
    if (mConfig.voxelFactor <= 0.0) {
        mpSpartanMap->voxelize(mConfig.voxelSize);
    } else {
        mpSpartanMap->voxelize(
                mConfig.voxelSize,
                mConfig.voxelSize,
                mConfig.voxelSize * mConfig.voxelFactor
        );
    }
    std::cout << "DONE" << std::endl;
    
    std::cout << "Truncating map to: " << mConfig.truncNorm << std::endl;
    mpSpartanMap->truncNorm(mConfig.truncNorm);
    std::cout << "DONE" << std::endl;

    std::cout << "Transforming map to world coordinates...\n";
    mpSpartanMap->transformPointCoords(total_tf);
    std::cout << "DONE" << std::endl;

    std::cout << "Denoising point cloud...\n";
    mpSpartanMap->filterPoints(
            mConfig.sniper1MinNeighbors,
            mConfig.sniper1GrowthFactor,
            mConfig.sniper1MinRadius,
            mConfig.sniper1MaxRadius,
            mConfig.sniper2MinNeighbors,
            mConfig.sniper2GrowthFactor,
            mConfig.sniper2MinRadius,
            mConfig.sniper2MaxRadius,
            mConfig.radNN,
            mConfig.radRadius);
    std::cout << "DONE" << std::endl;

    /* Logic can be added here to convert from the SpartanMap PCL
     * representation to whatever is required by CNES for external
     * interfacing. You can get the raw point cloud from the map
     * using the getters, probably getCloudPtr() is better since
     * it only returns geometric data and drops color information.
     * You can also embed the logic for the conversion in SpartanMap,
     * instead of doing it here, see the getTransportable() method
     * for an example using rock basetypes. */


    // Optionally also dump to an output file
    if (strlen(mConfig.dump_file_path) > 0
        && mCurrentFrame == 0   // WARNING: This is just for debugging!!!
    ) {
        std::cout << mConfig.dump_file_path
            << ", " << mCurrentFrame
            << std::endl;


        std::cout << "Printing map to output file...\n";
        mpSpartanMap->printToFile(
                mConfig.dump_file_path,
                mConfig.includeColors,
                true // this clears the output file before writing the new values
        );
        std::cout << "DONE" << std::endl;
    }

    mCurrentFrame++;
    std::cout << "DONE WITH MAPPING" << std::endl;
}


/**
 * Return a ROCK basetype for transporting the pointcloud
 * */
base::samples::Pointcloud MappingExecutor::getTransportable()
{
    return mpSpartanMap->getTransportable();
}


// This method is intended for use when logging. Specifically,
// by supplying a 0 or 1 for the mark argument one can get a
// Frame representation of the requested image in the state it
// was when vo_step() was last called via the nextPose() method.
base::samples::frame::Frame *MappingExecutor::getImgFrame(int mark)
{
    unsigned char *imgData;
    int width, height;
    if (mark == 0) {
        imgData = Limgdata;
        width = (int)(Lwidth * mConfig.horizontal);
        height = (int)(Lheight * mConfig.vertical);
    } else if (mark == 1) {
        imgData = Rimgdata;
        width = (int)(Rwidth * mConfig.horizontal);
        height = (int)(Rheight * mConfig.vertical);
    } else {
        std::cout
            << "Unreachable: MappingExecutor::getImgFrame"
            << std::endl;
        exit(1);
    }

    auto ret = new base::samples::frame::Frame(width, height);
    ret->setImage(imgData, width*height*sizeof(unsigned char));
    return ret;
}

