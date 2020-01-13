#include "SpartanMap.hpp"

using namespace spartan;
using namespace Eigen;
using namespace pcl;
using namespace std;

/**
 * Constructor for the class, converts from space sweep param object,
 * but obviously AFTER the mapping task has already been run
 * */
SpartanMap::SpartanMap(SweepParams *sp, double P0[NUM_PPARAMS], uint8_t *im0, int imWidth, int imHeight, int currentFrame)
{
    register int i, j;
    uint8_t outOfBoundsColor[3] = {0, 255, 0};
    int npoints = sp->dimVer * sp->dimHor;

    // WARNING: Apparently using std::make_shared<>() doesn't work,
    // need to use boost::make_shared instead. Also note that
    // this function FORWARDS arguments to the constructor, do not
    // explicitly invoke with new()!
    cloud = boost::make_shared<PointCloud<PointXYZRGB> >();

    // Loop through the points and if they are valid MNCC maxima
    // add them to the map and sample the color value that they
    // should get.
    for (i=0; i<npoints; i++) {
        if (sp->localMaxBoolMap[i] != 0) {
            PointXYZRGB pt;
            pt.x = sp->ptsMax[i][0];
            pt.y = sp->ptsMax[i][1];
            pt.z = sp->ptsMax[i][2];

            // Find reprojection on image to sample grayscale color
            double image_point[2];
            int impx, impy;
            projectOnImagePlane(P0, sp->ptsMax[i], image_point);

            impx = (int) image_point[0];
            impy = (int) image_point[1];

            // In case the point is beyond the image boundaries, the
            // color cannot be sampled, so we use some default
            if (image_point[0] < 0
                    || image_point[1] < 0
                    || image_point[0] >= imWidth
                    || image_point[1] >= imHeight) {
                pt.r = outOfBoundsColor[0];
                pt.g = outOfBoundsColor[1];
                pt.b = outOfBoundsColor[2];
            } else {
                uint8_t grayval = im0[impy*imWidth + impx];
                pt.r = grayval;
                pt.g = grayval;
                pt.b = grayval;
            }

            cloud->push_back(pt);
        }
    }
}

SpartanMap::~SpartanMap()
{
}


/**
 * Get a ROCK base-type that can be easily transported through
 * the framework
 * */
base::samples::Pointcloud SpartanMap::getTransportable()
{
    base::samples::Pointcloud result;
    for (size_t i=0; i<cloud->points.size(); i++) {
        base::Point iterPoint(
                cloud->points[i].x,
                cloud->points[i].y,
                cloud->points[i].z
        );
        result.points.push_back(iterPoint);
    }
    return result;
}

/**
 * Get PCL pointcloud with colors for each point
 * */
PointCloud<PointXYZRGB>::Ptr SpartanMap::getColoredCloudPtr()
{
    return cloud;
}


/**
 * Get PCL pointcloud without colors, only 3D coordinates per point.
 * */
PointCloud<PointXYZ>::Ptr SpartanMap::getCloudPtr()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr geo_cloud;
    copyPointCloud(*cloud, *geo_cloud);
    return geo_cloud;
}


/**
 * Transform the map pointcloud using some transform
 * */
void SpartanMap::transformPointCoords(Affine3d tf)
{

    auto tf_cloud(new PointCloud<PointXYZRGB>);
    transformPointCloud(*cloud, *tf_cloud, tf.cast<float>());
    tf_cloud->sensor_origin_ = tf.matrix().rightCols<1>().cast<float>();
    tf_cloud->sensor_orientation_ = tf.linear().cast<float>();
    cloud->swap(*tf_cloud);
}

void SpartanMap::statFilt(int mean_k, double stddev_multiplier)
{
    auto filt_cloud(new PointCloud<PointXYZRGB>);
    StatisticalOutlierRemoval<PointXYZRGB> stat_out_rem;
    stat_out_rem.setInputCloud(cloud);
    stat_out_rem.setMeanK(mean_k);
    stat_out_rem.setStddevMulThresh(stddev_multiplier);
    stat_out_rem.filter(*filt_cloud);
    cloud->swap(*filt_cloud);
}


/**
 * Same as SOR, but stat_order allows to use mean/stddev features extracted not
 * from Euclidean distances, but from Euclidean distances raised to the stat_order
 * exponent, meaning that stat_order > 1 creates stronger outlier detection but
 * potentially adds sensitivity to local noise.
 * */
void SpartanMap::flatFilt(int mean_k, double stddev_mult, double stat_order)
{
    auto filt_cloud(new PointCloud<PointXYZRGB>);
    KdTreeFLANN<PointXYZRGB> kdtree;
    kdtree.setInputCloud(cloud);
    vector<double> meanDists;
    for (size_t i=0; i<cloud->size(); i++) {
        vector<int> idxs;
        vector<float> dists;
        int neighbs;
        neighbs = kdtree.nearestKSearch((*cloud)[i], mean_k, idxs, dists);

        double avg_dist=0;
        for (const float d : dists) avg_dist += pow(static_cast<double>(d), 0.5*stat_order);
        avg_dist /= (mean_k - 1);
        meanDists.push_back(avg_dist);
    }
    double avgMeanDist=0, stdDevMeanDist=0;
    for (const double d : meanDists) {
        avgMeanDist += d;
        stdDevMeanDist += d*d;
    }
    stdDevMeanDist =
        (stdDevMeanDist - avgMeanDist * avgMeanDist / static_cast<double>(meanDists.size()))
        /(static_cast<double>(meanDists.size())-1);
    avgMeanDist /= static_cast<double>(meanDists.size());

    double dist_thresh = avgMeanDist + stdDevMeanDist*stddev_mult;
    for (size_t i=0; i<meanDists.size(); i++) {
        if (fabs(avgMeanDist - meanDists[i]) <= dist_thresh) {
            filt_cloud->push_back((*cloud)[i]);
        }
    }
    cloud->swap(*filt_cloud);

}


void SpartanMap::radFilt(int nNeighbors, double radius)
{
    auto filt_cloud(new PointCloud<PointXYZRGB>);
    RadiusOutlierRemoval<PointXYZRGB> rad_out_rem;
    rad_out_rem.setInputCloud(cloud);
    rad_out_rem.setMinNeighborsInRadius(nNeighbors);
    rad_out_rem.setRadiusSearch(radius);
    rad_out_rem.filter(*filt_cloud);
    cloud->swap(*filt_cloud);
}


void SpartanMap::sniperFilt(int minNeighbors, double growthRatio, double minRadius, double maxRadius)
{
    if (maxRadius < minRadius) {
        cerr << "Filtering with minimum radius=" << minRadius
            << " and maximum radius=" << maxRadius
            << ", which is illegal -> filtering ignored" << endl;
        return;
    }

    auto filt_cloud(new PointCloud<PointXYZRGB>);
    KdTreeFLANN<PointXYZRGB> kdtree;
    kdtree.setInputCloud(cloud);
    vector<int> searchIdxs;
    vector<float> sqDists;
    double sqMinRadius = minRadius * minRadius;
    for (size_t i=0; i<cloud->size(); i++) {
        int n1=0, n2;
        n2 = kdtree.radiusSearch((*cloud)[i], maxRadius, searchIdxs, sqDists);
        for (const float d : sqDists) {
            if (d < sqMinRadius) {
                n1++;   // Find #neighbors in smaller radius
            }
        }
        if (n1 * growthRatio <= n2
                && n1 >= minNeighbors) {    // apply radFilt and density check
            filt_cloud->push_back((*cloud)[i]);
        }
        searchIdxs.clear();
        sqDists.clear();
    }
    cloud->swap(*filt_cloud);
}


/**
 * Eliminates all points whose distance from the sensor origin is above a limit.
 * */
void SpartanMap::truncNorm(double normLimit)
{
    cout << "Size before truncation: " << cloud->size() << endl;
    auto trunc_cloud(new PointCloud<PointXYZRGB>);
    for (size_t i=0; i<cloud->size(); i++) {
        double xval = (*cloud)[i].x - cloud->sensor_origin_[0];
        double yval = (*cloud)[i].y - cloud->sensor_origin_[1];
        double zval = (*cloud)[i].z - cloud->sensor_origin_[2];
        if (xval * xval +
            yval * yval +
            zval * zval <=
            normLimit * normLimit) {
            trunc_cloud->push_back((*cloud)[i]);
        }
    }
    cloud->swap(*trunc_cloud);
    cout << "Size after truncation: " << cloud->size() << endl;
}


/**
 * Applies PCL standard statistical outlier filtering, parametrized
 * to the user's specifications
 * */
void SpartanMap::filterPoints(
        int s11, double s12, double s13, double s14,
        int s21, double s22, double s23, double s24,
        int r1, double r2)
{
    cout << "SIZE BEFORE FILTERING: " << cloud->size() << endl;
    //sniperFilt(5, 5.5, 0.1, 0.3);
    sniperFilt(s11, s12, s13, s14);
    cout << "SIZE AFTER FILTER #1: " << cloud->size() << endl;
    //sniperFilt(4, 6, 0.05, 0.15);
    sniperFilt(s21, s22, s23, s24);
    cout << "SIZE AFTER FILTER #2: " << cloud->size() << endl;
    //radFilt(4, 0.05);
    radFilt(r1, r2);
    cout << "SIZE AFTER FILTER #3: " << cloud->size() << endl;
}


/**
 * Converts the dense point cloud to a sparser voxel grid to represent
 * a DEM
 * */
void SpartanMap::voxelize(double vlsx, double vlsy, double vlsz)
{
    auto vox_cloud(new PointCloud<PointXYZRGB>);
    VoxelGrid<PointXYZRGB> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(vlsx, vlsy, vlsz);
    vg.filter(*vox_cloud);
    cloud->swap(*vox_cloud);
}


/**
 * Same as above, but uses cubical voxels
 * */
void SpartanMap::voxelize(double vls)
{
    voxelize(vls, vls, vls);
}


/**
 * Prints maps to file according to the specifications given. If clearFile is false,
 * appends to the existing file instead of overwriting.
 * */
void SpartanMap::printToFile(char filePath[], bool includeColors, bool clearFile, char delim[])
{
    // Try opening the output file
    ofstream dumpFile(filePath, (clearFile ? ios::trunc : ios::app));
    if (! dumpFile.good()) {
        cerr << "Mapping output file could not be opened!" << endl;
        exit(1);
    }

    // Define a format string for each line in the output file,
    // depending on whether color info is required
    char line_fmt[MAX_STR_LEN], line_wc[] = "%f%s%f%s%f%s%d%s%d%s%d", line_woc[] = "%f%s%f%s%f",
         line[MAX_STR_LEN];

    if (includeColors) {
        strcpy(line_fmt, line_wc);
        cout << "WITH COLORS: " << strlen(line_fmt);
    } else {
        strcpy(line_fmt, line_woc);
        cout << "WITHOUT COLORS: " << strlen(line_fmt);
    }

    // Loop over all points and write the appropriate line in the output
    // file
    register int i;
    for (i=0; i<cloud->width; i++) {
        if (!includeColors) {
            sprintf(line, line_fmt,
                    (*cloud)[i].x,
                    delim,
                    (*cloud)[i].y,
                    delim,
                    (*cloud)[i].z);
        } else {
            sprintf(line, line_fmt,
                    (*cloud)[i].x,
                    delim,
                    (*cloud)[i].y,
                    delim,
                    (*cloud)[i].z,
                    delim,
                    (*cloud)[i].r,
                    delim,
                    (*cloud)[i].g,
                    delim,
                    (*cloud)[i].b);
        }
        dumpFile << line << endl;
    }

    // Close the file
    dumpFile.close();

    cout << "--- PRINT FINISHED ---" << endl;
}

