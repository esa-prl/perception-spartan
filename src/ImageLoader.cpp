#include "ImageLoader.hpp"

#define DEBUG

using namespace std;
using namespace base::samples::frame;
using namespace spartan;

ImageLoader::ImageLoader(const OdometryConfig& odom_conf)
{
    spartan_width = odom_conf.spartanWidth;
    spartan_height = odom_conf.spartanHeight;
    strcpy(calib_full_path, odom_conf.calib_path);

    cout << "CALIB PATH: " << calib_full_path << endl;
    cout << "CALIB PATH RCVD: " << odom_conf.calib_path << endl;

    uid_counter = 0;
    lfc = 0; rfc = 0;
    left_fresh = false;
    right_fresh = false;
    frame_left = NULL;
    frame_right = NULL;
    getFinalMatrices();
}

ImageLoader::~ImageLoader()
{
    delete frame_left;
    delete frame_right;
}


/**
 * This method returns a packaged version of the calibration information
 * that is compatible with OdometryExecutor's constructor.
 * */
CalibInfo ImageLoader::getCalibInfo()
{
    CalibInfo ret;
    exportUpdatedCalibrationParams(p2, ret.K_LEFT, ret.K_RIGHT,
            ret.D_LEFT, ret.D_RIGHT, ret.ROT_RIGHT, ret.T_RIGHT);
    return ret;
}

/**
 * Returns a reasonable size by minimally editing the size of the original
 * image to match the target aspect ratio
 * */
cv::Size ImageLoader::getIntermediateSize()
{
	double aspect_ratio_orig = ((double) orig_width) / orig_height;
	double aspect_ratio_tar = ((double) spartan_width) / spartan_height;

	double width = orig_width, height = orig_height;

	// Only edit one dimension, e.g. if we're narrowing an image, only
	// width needs to change
	if (aspect_ratio_orig > aspect_ratio_tar) {
		width = height * aspect_ratio_tar;
	} else {
		height = width / aspect_ratio_tar;
	}

#ifdef DEBUG
    cout << "--- cv::INTERM SZ ---" << endl;
    cout << cvRound(width) << ", " << cvRound(height) << endl;
#endif

	return cv::Size(cvRound(width), cvRound(height));
}


/**
 * This method initializes the mappings necessary for image undistortion,
 * rectification and aspect ratio adaptation
 * */
void ImageLoader::getFinalMatrices()
{

    // Read original calibration file
    cv::FileStorage fs(calib_full_path, cv::FileStorage::READ);
    cv::Mat c1, c2, d1, d2, t, r;
    fs[CALIB_IN_IMG_WIDTH] >> orig_width;
    fs[CALIB_IN_IMG_HEIGHT] >> orig_height;
    fs[CALIB_IN_CAMERA_MAT_LEFT] >> c1;
    fs[CALIB_IN_CAMERA_MAT_RIGHT] >> c2;
    fs[CALIB_IN_DIST_COEFFS_LEFT] >> d1;
    fs[CALIB_IN_DIST_COEFFS_RIGHT] >> d2;
    fs[CALIB_IN_ROT_MAT] >> r;
    fs[CALIB_IN_TRANS_COEFFS] >> t;

#ifdef DEBUG
    cout << "--- INPUT FROM CALIB FILE ---" << endl;
    cout << c1 << endl;
    cout << c2 << endl;
    cout << d1 << endl;
    cout << d2 << endl;
    cout << r << endl;
    cout << t << endl;
#endif

    // Get matrices and maps for rectification and aspect ratio correction
	intermediate_sz = getIntermediateSize();
    cv::Mat r1, r2, q;
    cv::stereoRectify(
            c1, d1, c2, d2, cv::Size(orig_width, orig_height), r, t, // inputs
            r1, r2, p1, p2, q,   // outputs
            cv::CALIB_ZERO_DISPARITY, 0, intermediate_sz   // defaults -> use alpha=0 to avoid black edges in the output image
    );

    cv::initUndistortRectifyMap(
            c1, d1, r1, p1, intermediate_sz, CV_32FC1, //inputs
            lm1, lm2  //outputs
    );
    cv::initUndistortRectifyMap(
            c2, d2, r2, p2, intermediate_sz, CV_32FC1, //inputs
            rm1, rm2  //outputs
    );
}


/**
 * This method exports the updated equivalent calibration parameters to preallocated
 * memory blocks, specifically:
 * K -> 9 doubles
 * D -> 5 doubles
 * R -> 9 doubles
 * T -> 3 doubles
 * */
void ImageLoader::exportUpdatedCalibrationParams(cv::Mat proj_mat_right, double *KL, double *KR, double *DL, double *DR, double *R, double *T)
{

	// First calculate the new calibration data
	// Start by computing the scaling factors for final resizing
	double xfactor = ((double) spartan_width) / intermediate_sz.width,
		yfactor = ((double) spartan_height) / intermediate_sz.height;
	cv::Mat final_K;
	proj_mat_right(cv::Rect(0,0, 3,3)).copyTo(final_K);
	for (int j=0; j<3; j++) {
		final_K.at<double>(0, j) *= xfactor;
		final_K.at<double>(1, j) *= yfactor;
	}

	// Then use the rectification property to find
	// the equivalent baseline translation
	cv::Mat dist_coeffs = cv::Mat::zeros(cv::Size(5, 1), CV_64FC1);
	cv::Mat rot_mat = cv::Mat::zeros(cv::Size(3, 3), CV_64FC1);
	cv::Mat trans_coeffs = cv::Mat::zeros(cv::Size(1, 3), CV_64FC1);
	double fx = proj_mat_right.at<double>(0, 0),
		dx = proj_mat_right.at<double>(0, 3);
	trans_coeffs.at<double>(0, 0) = dx / fx;

    memcpy(KL, (double*)final_K.data, 9*sizeof(double));
    memcpy(KR, (double*)final_K.data, 9*sizeof(double));
    memcpy(DL, (double*)dist_coeffs.data, 5*sizeof(double));
    memcpy(DL, (double*)dist_coeffs.data, 5*sizeof(double));
    memcpy(R, (double*)rot_mat.data, 9*sizeof(double));
    memcpy(T, (double*)trans_coeffs.data, 3*sizeof(double));
}


/**
 * This method updates one of the stored frames, depending on
 * the frame feed code, storing a grayscale equivalent locally.
 * */
void ImageLoader::newFrame(int frame_feed_code, Frame *fr)
{
    //cout << "IL: GOT NEW FRAME - "
    //    << ((frame_feed_code == LEFT_CAMERA_FEED)
    //            ? "LEFT "
    //            : "RIGHT ")
    //    << ((frame_feed_code == LEFT_CAMERA_FEED)
    //            ? lfc
    //            : rfc)
    //    << ", " << (int)(fr->getImagePtr()[0])
    //    << ", " << (int)(fr->getImagePtr()[100])
    //    << ", " << (int)(fr->getImagePtr()[200])
    //    << endl;
    // Get a grayscale representation of the frame first
    Frame *new_frame = getGrayscaleEquiv(*fr);

    // Check for left VS right and update accordingly
    if (frame_feed_code == LEFT_CAMERA_FEED) {
        if (frame_left != NULL)  {
            delete frame_left;
            frame_left = NULL;
        }
        frame_left = new_frame;
        lfc++; left_fresh = true;
    } else if (frame_feed_code == RIGHT_CAMERA_FEED) {
        if (frame_right != NULL) {
            delete frame_right;
            frame_right = NULL;
        }
        frame_right = new_frame;
        rfc++; right_fresh = true;
    } else {
        throw invalid_argument("camera feed argument invalid, check ImageLoader.hpp for the definitions");
    }

    // In the end, this method should store the new frame in the
    // appropriate position, freeing the memory allocated for the
    // old one and setting the corresponding "fresh" variable to
    // true (the counter also increments for potential future use)

    //cout << "IL: DONE WITH NEW FRAME - "
    //    << ((frame_feed_code == LEFT_CAMERA_FEED)
    //            ? "LEFT "
    //            : "RIGHT ")
    //    << ((frame_feed_code == LEFT_CAMERA_FEED)
    //            ? lfc
    //            : rfc)
    //    << ", " << (int)(new_frame->getImagePtr()[0])
    //    << ", " << (int)(new_frame->getImagePtr()[100])
    //    << ", " << (int)(new_frame->getImagePtr()[200])
    //    << endl;
}

/**
 * This method returns a grayscale version of an initial Frame,
 * converting appropriately in case RGB is given instead.
 * */
Frame *ImageLoader::getGrayscaleEquiv(Frame& fin)
{
    // If already in grayscale, do nothing
    if (fin.isGrayscale())
        return new Frame(fin);

    // Assuming that RGB is the only alternative (manually code
    // something else if you need it)
    if (! fin.isRGB()) {
        char err[100];
        sprintf(err, "expected MODE_GRAYSCALE or MODE_RGB, got mode: %d, see enum at Frame.hpp for definitions", static_cast<int>(fin.getFrameMode()));
        throw invalid_argument(string(err));
    }

    // Allocate new frame
    Frame *res = new Frame(
            fin.getWidth(),
            fin.getHeight(),
            8,
            MODE_GRAYSCALE,
            0,

            // Divides by 3 because of RGB, change if necessary
            // for different frame type
            fin.getNumberOfBytes()/3);

    // Explicitly convert image data from RGB to grayscale
    std::vector<uint8_t> gray_img = grayscalify(fin.getImage());

    // Set frame image and time
    res->setImage(gray_img);
    res->time = fin.time;

    return res;
}


/**
 * This method takes an rgb_image (assumed here to be a vector
 * of uint8_t entries, with entries [3k, 3k+1, 3k+2] corresponding
 * to 1 pixel) and outputs one grayscale equivalent, based
 * on the SPARTAN conversion spec:
 *      <GRAY> =
 *          0.30 * <RED> +
 *          0.59 * <GREEN> +
 *          0.11 * <BLUE>
 * */
std::vector<uint8_t> ImageLoader::grayscalify(std::vector<uint8_t> rgb_image)
{
    std::vector<uint8_t> res = std::vector<uint8_t>(rgb_image.size()/3);
    for (unsigned int i=0; i<rgb_image.size(); i+=3)
    {
        res[i/3] = (uint8_t)((
            (int)rgb_image[i] * 77 +
            (int)rgb_image[i+1] * 151 +
            (int)rgb_image[i+2] * 28)
            >> 8);  // Use shifting for efficiency
    }

    return res;
}


/**
 * Simple method to check whether the two current frames are
 * withing an appropriate time distance of each other, where
 * "appropriate" means below the threshold specified in the
 * configuration.
 * */
bool ImageLoader::timestampThreshCheck()
{
    return true;
        //abs(
        //        frame_left->time.toMilliseconds() -
        //        frame_right->time.toMilliseconds()
        //) < TIME_STAMP_THRESH_MILLI;
}


/**
 * This function applies corrections to the frame data
 * */
void ImageLoader::applyOpticalCorrections(Frame& lf, Frame& rf)
{
    cv::Mat left_img(
            cv::Size(frame_left->getWidth(), frame_left->getHeight()),
            CV_8UC1,
            frame_left->getImagePtr()
    );
    cv::Mat right_img(
            cv::Size(frame_right->getWidth(), frame_right->getHeight()),
            CV_8UC1,
            frame_right->getImagePtr()
    );

    cv::Mat left_new, right_new;
    remap(left_img, left_new, lm1, lm2, cv::INTER_LINEAR);
    remap(right_img, right_new, rm1, rm2, cv::INTER_LINEAR);

    bool zooming_in = (left_img.rows < spartan_height);
    cv::Mat left_final, right_final;
    resize(left_new, left_final, cv::Size(spartan_width, spartan_height), 0, 0,
            (zooming_in ? cv::INTER_CUBIC : cv::INTER_AREA));
    resize(right_new, right_final, cv::Size(spartan_width, spartan_height), 0, 0,
            (zooming_in ? cv::INTER_CUBIC : cv::INTER_AREA));

    lf.setImage((uint8_t*)left_final.data, spartan_width*spartan_height*sizeof(uint8_t));
    rf.setImage((uint8_t*)right_final.data, spartan_width*spartan_height*sizeof(uint8_t));
}


/**
 * This function returns true if the cached pair in the ImageLoader
 * is usable, meaning its frames are both fresh and the timestamp
 * difference between them is below the specified threshold.
 *
 * As an extra guarantee, the frames are said to be "synced" if
 * the left and right counters are equal. This is important in
 * the case of:
 * left frame @ t1
 * right frame @ t2
 * |t1 - t2| > thresh
 * new left frame @t3
 * |t2 - t3| < thresh
 * The (t3,t2) pair would be accepted, since both frames are
 * fresh AND their timestamps are OK. But their counters are
 * out of sync.
 * */
bool ImageLoader::properPairStored()
{
    return left_fresh &&
        right_fresh &&
        //lfc == rfc && // uncomment to activate the extra guarantee
        timestampThreshCheck();
}

/**
 * This method returns a new frame pair when requested, but only
 * if this pair is safe to use. If we are at startup or the
 * incoming frames have diverged from the specifications set
 * then the method returns NULL to avoid working on spurious
 * results.
 * */
FramePair *ImageLoader::getFramePair()
{
    if (frame_left == NULL ||
            frame_right == NULL) {
        std::cout << "Requested pair, but startup not yet finished, returning NULL!"
            << std::endl;
        return NULL;
    }

    if (! properPairStored()) {
        //std::cout << "Pair is not yet ready: "
            //<< lfc << ", "
            //<< rfc << std::endl;
        return NULL;
    }

    //if (uid_counter%REJECT_BATCH != 0) {
    //    uid_counter++;
    //    return NULL;
    //}

    // FramePair is defined in Frame.hpp
    FramePair *fp = new FramePair();
    fp->time = base::Time();
    Frame left_final(spartan_width, spartan_height), right_final(spartan_width, spartan_height);
    applyOpticalCorrections(left_final, right_final);
    fp->first = Frame(left_final);
    fp->second = Frame(right_final);
    fp->id = uid_counter++;

    // Both images are now stale, must get new ones before sending
    left_fresh = false;
    right_fresh = false;

    return fp;
}
