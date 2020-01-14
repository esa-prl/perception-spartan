#include "OdometryExecutor.hpp"

using namespace spartan;
using namespace std;
using namespace Eigen;
using namespace cv;

OdometryExecutor::OdometryExecutor(const OdometryConfig& conf, CalibInfo ci, Eigen::Affine3d cam2body_tf)
{
    mConfig = conf;
    int ret;
    vo_initialized = false;
    last_pair_uid = -1;

    /* TRANSFORM INITIALIZATIONS */

    /* Exoter -> This specific transform can be received from the ROCK Transformer,
     * instead of being hard-coded */
    //Quaterniond q_base;
    //double rot_z = -M_PI / 2.0;
    //double rot_x = -M_PI / 180.0 * 120.4;
    //q_base = AngleAxisd(rot_z, Vector3d::UnitZ())
    //    * AngleAxisd(rot_x, Vector3d::UnitX());
    //Vector3d t_base(0.2038, 0.0685, 0.2402);
    devon=false;


    /* Devon */
    /*
    Quaterniond q_base;
    double rot_mag = -2.8123980539;
    Vector3d axis_orig(-0.2660987251, -0.7639785818, 0.5878164638);
    q_base = AngleAxisd(rot_mag, axis_orig);
    Vector3d t_base(Vector3d::Zero());
    devon=true;
    */

    /* Morocco */
    // From all_metadata.txt, @[32-38], constant for all lines: X Y Z - QW QX QY QZ
    //0.5003 0.0816 0.6637 0.3631086859 -0.605320796 0.6081407716 -0.3631853771
    /*
    Quaterniond q_base(0.3631086859, -0.605320796, 0.6081407716, -0.3631853771);
    Vector3d t_base(0.5003, 0.0816, 0.6637);
    devon = false;
    */

    // Base transform -> Use this block in case you are hard-coding the transform
    // instead of receiving it from the ROCK Transformer

    // ------ for the PTU  the body2cam transform is given by: body->mast, mast->ptu, ptu->cam
    // base_tf (body->cam) = ptu2cam_tf*mast2ptu_tf*boby2mast_tf
    Eigen::Affine3d body2mast_tf;
    body2mast_tf = Eigen::Affine3d::Identity();
    Vector3d body2mast_t(0.078, 0.0, 0.633);
    Eigen::Quaternion<double> body2mast_q(1.0, 0.0, 0.0, 0.0);
    body2mast_tf.translate(body2mast_t);
    body2mast_tf.rotate(body2mast_q);

    Eigen::Affine3d mast2ptu_tf;
    mast2ptu_tf = Eigen::Affine3d::Identity();
    Vector3d mast2ptu_t(0.0, 0.0, 0.03425);
    Eigen::Quaternion<double> mast2ptu_q(0.965926, 0.0, 0.258817, 0.0);
    mast2ptu_tf.rotate(mast2ptu_q);
    mast2ptu_tf.translate(mast2ptu_t);

    Eigen::Affine3d ptu2cam_tf;
    ptu2cam_tf = Eigen::Affine3d::Identity();
    Vector3d ptu2cam_t(-0.02, 0.06, 0.094);
    Eigen::Quaternion<double> ptu2cam_q(0.5, 0.5, -0.5, -0.5);
    ptu2cam_tf.translate(ptu2cam_t);
    ptu2cam_tf.rotate(ptu2cam_q);

    base_tf = Eigen::Affine3d::Identity();
    base_tf = ptu2cam_tf*mast2ptu_tf*body2mast_tf;
    // ------

    // ------ bruteforcy way of writing the 4d matrix and pushing it inside the Affine3d transform
    Matrix4d M;
    M << 0.0,   -0.512043,  0.85896,    0.1124,
         -1.0,  0.0,        0.0,        0.06,
         0.0,   -0.85896,    -0.512043,  0.6530,
         0.0,   0.0,        0.0,        1.0;
    base_tf = Eigen::Affine3d::Identity();
    base_tf.matrix() = M;
    // ------

    std::cout << "base_tf: " << std::endl << base_tf.matrix() << std::endl;
    std::cout << "cam2body_tf arg: " << std::endl << cam2body_tf.matrix() << std::endl;

    //Vector3d sec_off(1000, 1000, 1000);
    //sec_round_offset = Affine3d::Identity();
    //sec_round_offset.translate(sec_off);
    //inv_sec_round_offset = sec_round_offset.inverse();

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
    KLC[2] -= mConfig.spartanWidth * (1.0-mConfig.horizontal);
    KLC[5] -= mConfig.spartanHeight * (1.0-mConfig.vertical-mConfig.shiftBot);
    memcpy(distort_coeffs_left, ci.D_LEFT, NUM_DISTORT*sizeof(ci.D_LEFT[0]));

    // Copy over the rotation matrix (3x3 = 9 doubles)
    memcpy(rot_mat, ci.ROT_LEFT, 9*sizeof(ci.ROT_LEFT[0]));

    // Now call the SAM library function to do the convertion, see
    // <SPARTAN_INSTALL_DIR>/software/vomap/src/VO/sam.c for implementation
    sam_rotmat2vec(rot_mat, r);

    memcpy(t, ci.T_LEFT, 3*sizeof(ci.T_LEFT[0]));
    sam_PfromKrt(KLC, r, t, LPmat);

    // Do the same for the right camera
    memcpy(K_RIGHT, ci.K_RIGHT, 9*sizeof(ci.K_RIGHT[0]));
    memcpy(KRC, ci.K_RIGHT, 9*sizeof(ci.K_RIGHT[0]));
    KRC[5] -= mConfig.spartanHeight * (1.0-mConfig.vertical-mConfig.shiftBot);
    memcpy(distort_coeffs_right, ci.D_RIGHT, NUM_DISTORT*sizeof(ci.D_RIGHT[0]));
    memcpy(rot_mat, ci.ROT_RIGHT, 9*sizeof(ci.ROT_RIGHT[0]));
    sam_rotmat2vec(rot_mat, r);
    memcpy(t, ci.T_RIGHT, 3*sizeof(ci.T_RIGHT[0]));
    sam_PfromKrt(KRC, r, t, RPmat);

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
    for (double x : LPmat)
        std::cout << x << ", ";
    std::cout << std::endl;
    std::cout << "RIGHT\n";
    for (double x : RPmat)
        std::cout << x << ", ";
    std::cout << std::endl;
}


OdometryExecutor::~OdometryExecutor()
{
    free(Limgdata);
    free(Rimgdata);
    vo_cleanup(vos);
    free(gtPoses);
}


/**
 * This method initializes the VO-state struct. The functionality
 * could be extended with vo_initpose() if necessary.
 * */
void OdometryExecutor::initializeVisOdomAgent(uint16_t width, uint16_t height)
{
    vos=vo_init(LPmat, RPmat, width, height);
    //Vector3d sro_tr = sec_round_offset.translation();
    //double rt_offset[NUM_RTPARAMS] = {0,0,0,sro_tr(0), sro_tr(1), sro_tr(2)};
    //vo_initpose(vos, rt_offset);

    // After the actual initialization, keep track that this is
    // done with the vo_initialized flag
    vo_initialized = true;
    return;
}


/**
 * This method is uded internally to check whether undistortion
 * parameters were read from the calibration files that are
 * larger than some very small threshold, meaning that bouguet
 * undistortion is necessary.
 * */
bool OdometryExecutor::undistortParamsPresent()
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
void OdometryExecutor::cropTopAndLR(uint8_t *orig_img, int orig_width, int orig_height, bool isLeft, int final_width, int final_height, int shift_bot, uint8_t *res)
{
    //std::cout << "CROPPING" << std::endl;
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
    //std::cout << "CROPPED" << std::endl;
    return;
}


/**
 * This method returns the next pose that VO calculates, given the
 * current one and a pair of frames captured after its computation
 * */
void OdometryExecutor::nextPose(base::samples::frame::FramePair stereo_pair, vector<double>& centeredPose)
{
    // If the frame pair is repeated, drop it as an echo
    if (stereo_pair.id == last_pair_uid) {
        std::cout << "Repeated frame pair ID, ignoring" << std::endl;
        return;
    }

    bool clear_dump_file = (last_pair_uid == -1);
    last_pair_uid = (int) stereo_pair.id;

    unsigned char *Limg_distorted, *Rimg_distorted;

    // Get the data, assumed to be grayscale by now
    Limg_distorted = stereo_pair.first.getImagePtr();
    Rimg_distorted = stereo_pair.second.getImagePtr();

    // If not initialized, infer parameters from the frame data
    // and initialize accordingly
    if (! vo_initialized) {
        // Get frame size
        base::samples::frame::frame_size_t frame_size_first =
            stereo_pair.first.getSize();
        base::samples::frame::frame_size_t frame_size_second =
            stereo_pair.first.getSize();
        // Basic requirement: Frame sizes should match EXACTLY
        // between left and right (and are assumed to remain
        // constant throughout any single execution)
        if (frame_size_first != frame_size_second) {
            std::cout << "ERROR: incompatible camera feed sizes!" << std::endl;
            exit(1);
        }
        Lwidth = frame_size_first.width;
        Rwidth = Lwidth;
        Lheight = frame_size_first.height;
        Rheight = Lheight;

        initializeVisOdomAgent((int)(Lwidth*mConfig.horizontal), (int)(Lheight*mConfig.vertical));
    }

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
        //std::cout << "UNDISTORTING" << std::endl;
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

    //std::cout << "REALLOC" << std::endl;
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


    // Do one VO step. This is where FPGA acceleration may be
    // occurring behind the scenes.
    vo_step(
            vos,
            Limgdata,
            Rimgdata,
            rt,
            NULL,
            pts3D,
            NULL,
            NULL,
            &npts3D
    );

    // Odometry computed pose:
    // SPARTAN assumes the original frame captured by the camera
    // to be the origin. At each iteration vo_step returns the
    // transformation from that camera frame at t=0 to the current
    // frame. This frame is oriented so that:
    // - Z points out of the camera lens and into the terrain
    // - Y points down in the image
    // So if the camera is facing forward and tilted down toward the
    // ground, a forward movement of the rover would result in
    // positive Z and negative Y local displacement, nothing
    // along X.
    sam_vec2quat(rt, qrt);
    qrt[4] = rt[3];
    qrt[5] = rt[4];
    qrt[6] = rt[5];

    /* This is the pose corresponding to the transform from the
     * original camera frame to the current one. Because of the
     * original camera frame's orientation, this is NOT the rover
     * pose that one might expect, it still needs to be further
     * transformed */
    vector<double> raw_res(qrt, qrt+sizeof(qrt)/sizeof(qrt[0]));

    /* CAUTION: Map<Quaternion<...> > accepts list of quaternion
     * components in order X,Y,Z,W - NOT W,X,Y,Z as is normal*/
    double qq_raw[4] = {qrt[1], qrt[2], qrt[3], qrt[0]};
    double tt_raw[3] = {qrt[4], qrt[5], qrt[6]};
    Map<Quaternion<double> > qq(qq_raw);
    Map<Matrix<double, 1,3,RowMajor> > tt(tt_raw);
    Eigen::Affine3d camera_cumulative_tf(Eigen::Affine3d::Identity());
    camera_cumulative_tf.translate(tt.transpose());
    camera_cumulative_tf.rotate(qq);

    /* The idea is to define a translational offset to avoid
     * rounding errors near 0. This offset is introduced at
     * the initialization of the VO and then needs to be inverted,
     * however just calling .inverse() is insufficient, because
     * the frame has rotated slightly. So we need to first project
     * the offset in the current frame of reference of the camera
     * and then actually use the inv_sec_round_offset which we
     * precomputed at the constructor.
     *
     * Final matrix computation:
     * (INITIAL_OFFSEAT * CAM_2_CAM) <- this is SPARTAN's output
     * => (SPARTAN) * INVERSE_SPARTAN_ROT * INVERSE_OFFSET
     *
     * */
    // Undo rounding offset correction
    //Affine3d counter_transform(Affine3d::Identity());
    //counter_transform.rotate(camera_cumulative_tf.linear().inverse());
    //camera_cumulative_tf =
        //camera_cumulative_tf *
        //counter_transform *
        //inv_sec_round_offset;


    // These lines create a transform that expresses the final
    // body frame as a transform with respect to the original
    // body frame. To do this the steps are:
    // A = BODY_0   ->  CAMERA_0 (base_tf)
    // B = CAMERA_0 ->  CAMERA_t (SPARTAN transform, tt and qq)
    // C = CAMERA_t ->  BODY_t (inverted base_tf) = A^(-1)
    // Total matrix is A*B*A^(-1), which means that when SPARTAN
    // outputs B as the camera-to-camera transform, we convert
    // it to a body-to-body transform using the above relation,
    // since A is given (calculated at the constructor, values
    // come from the geometry of the rover and the camera mount).
    Eigen::Affine3d tf(base_tf);
    //tf.translate(tt.transpose());   // First translate, then rotate
    //tf.rotate(qq);
    tf = tf * camera_cumulative_tf;
    if (!devon) {
        tf = tf * inv_base_tf;  // Inverse precomputed for efficiency
    } // For devon semantics, we now have camera poses expressed in topocentric frame.

    double pos_final[3] = {
        tf.translation().data()[0],
        tf.translation().data()[1],
        tf.translation().data()[2]
    };

    dumpPoseToFile(pos_final, mConfig.dump_file_path, clear_dump_file);

    Quaterniond qq_final;
    qq_final = tf.linear();
    double quat_final[4] = {
        qq_final.w(),
        qq_final.x(),
        qq_final.y(),
        qq_final.z()
    };


    /* Store result of VO for additional return */
    // First come the (w,x,y,z) quaternion components
    for (int i=0; i<4; i++) centeredPose[i] = quat_final[i];
    // Then the x,y,z translation components
    for (int i=4; i<7; i++) centeredPose[i] = pos_final[i-4];

    base::samples::RigidBodyState rbs_new;
    rbs_new.setTransform(tf);
    //rbs_new.time = base::Time::now();
    rbsRegistry.push_back(rbs_new);
    //std::cout << "DONE WITH NEXTPOSE" << std::endl;
}


/**
 * Function for devon route plotting data extraction
 * */
void OdometryExecutor::dumpPoseToFile(double pose[], char *filepath, bool clear_file)
{
    ofstream dump;
    dump.open(filepath, ios::out | ((clear_file) ? ios::trunc : ios::app));
    dump << pose[0] << '\t' << pose[1] << '\t' << pose[2] << std::endl;
    dump.close();
}

// This method returns the VO-state internal object, in case some
// custom query needs to be made. Normally should not be used.
struct vostate *OdometryExecutor::getVisOdomState()
{
    return vos;
}


// This method is intended for use when logging. Specifically,
// by supplying a 0 or 1 for the mark argument one can get a
// Frame representation of the requested image in the state it
// was when vo_step() was last called via the nextPose() method.
base::samples::frame::Frame *OdometryExecutor::getImgFrame(int mark)
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
            << "Unreachable: OdometryExecutor::getImgFrame"
            << std::endl;
        exit(1);
    }

    auto ret = new base::samples::frame::Frame(width, height);
    ret->setImage(imgData, width*height*sizeof(unsigned char));
    return ret;
}

/**
 * Simple utility to get the last pose calculated by the
 * algorithm in the ROCK standardized RigidBodyState type.
 * The returned value is the last pushed_back to the
 * path vector updated by the nextPose() method.
 * */
base::samples::RigidBodyState OdometryExecutor::getRBS()
{
    return rbsRegistry.back();
}

