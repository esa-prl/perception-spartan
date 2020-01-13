#ifndef _SPARTAN_MAP_HPP_
#define _SPARTAN_MAP_HPP_

#include <iostream>
#include <fstream>
#include <vector>
#include <string>

#include <Eigen/Dense>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <base/samples/Pointcloud.hpp>

/* SPARTAN INCLUDES */
extern "C" {
#include <sweep/cameraGeometry.h>
#include <sweep/geometry.h>
#include <typesnconsts.h>
#include <matrixOps.h>
#include <debugfileIO.h>
#include <sweepFileIO.h>
#include <vgg_interp2.h>
#include <spaceSweep.h>
}
#include <lodepng.h>
/********************/

#define MAX_STR_LEN 256

using namespace std;
using namespace Eigen;
using namespace pcl;

namespace spartan
{

    class SpartanMap
    {
        protected:
            vector<double> geometricMap;
            vector<vector<uint8_t> > colorMap;

            PointCloud<PointXYZRGB>::Ptr cloud;
            void statFilt(int, double);
            void radFilt(int, double);
            void sniperFilt(int, double, double, double);
            void flatFilt(int, double, double);

        public:
            SpartanMap(SweepParams *, double[], uint8_t*, int, int, int);
            ~SpartanMap();
            void transformPointCoords(Affine3d);
            void filterPoints(
                    int, double, double, double,
                    int, double, double, double,
                    int, double);
            void truncNorm(double);
            void printToFile(char[], bool includeColors=false, bool clearFile=false, char delim[]="\t");
            void voxelize(double, double, double);
            void voxelize(double);
            PointCloud<PointXYZRGB>::Ptr getColoredCloudPtr();
            PointCloud<PointXYZ>::Ptr getCloudPtr();
            base::samples::Pointcloud getTransportable();
    };  // end class SpartanMap

} // end namespace spartan

#endif // _SPARTAN_MAP_HPP_
