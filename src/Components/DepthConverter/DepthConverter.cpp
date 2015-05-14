/*!
 * \file
 * \brief
 * \author Dawid Kaczmarek
 */

#include <memory>
#include <string>

#include "DepthConverter.hpp"
#include "Common/Logger.hpp"
#include <Types/MatrixTranslator.hpp>

#include <boost/bind.hpp>
#include <boost/format.hpp>

namespace Processors {
namespace DepthConventer {

DepthConventer::DepthConventer(const std::string & name) :
        Base::Component(name)
{
}

DepthConventer::~DepthConventer() {
}

void DepthConventer::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_rgb_stereo", &in_rgb_stereo);
	registerStream("in_depth_xyz", &in_depth_xyz);
	registerStream("out_cloud_xyzrgb", &out_cloud_xyzrgb);

	// Register handlers
	registerHandler("CalculateDepthMap", boost::bind(&DepthConventer::CalculateDepthMap, this));
	addDependency("CalculateDepthMap", &in_rgb_stereo);
	addDependency("CalculateDepthMap", &in_depth_xyz);
}

bool DepthConventer::onInit() {
	return true;
}

bool DepthConventer::onFinish() {
	return true;
}

bool DepthConventer::onStop() {
	return true;
}

bool DepthConventer::onStart() {
	return true;
}

void DepthConventer::CalculateDepthMap() {
	LOG(LINFO) << "Init CalculateDepthMap";
    cv::Mat oLeftRectified(in_rgb_stereo.read());
    cv::Mat xyz(in_depth_xyz.read());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    uint32_t pr, pg, pb;
    const double max_z = 1.0e4;

    LOG(LINFO) << "Generating depth point cloud";
    try {
        for(int y = 0; y < xyz.rows; y++)
        {
            uchar* rgb_ptr = oLeftRectified.ptr<uchar>(y);
            for(int x = 0; x < xyz.cols; x++)
            {
                cv::Vec3f point = xyz.at<cv::Vec3f>(y, x);
                if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;


                //Get RGB info
                pb = rgb_ptr[3*x];
                pg = rgb_ptr[3*x+1];
                pr = rgb_ptr[3*x+2];

                //Insert info into point cloud structure
                pcl::PointXYZRGB point1;
                point1.x = point[0];
                point1.y = point[1];
                point1.z = point[2];
                uint32_t rgb = (static_cast<uint32_t>(pr) << 16 | static_cast<uint32_t>(pg) << 8 | static_cast<uint32_t>(pb));
                point1.rgb = *reinterpret_cast<float*>(&rgb);
                cloud->push_back(point1);
            }
        }

        LOG(LINFO) << "Writing to data stream";
        out_cloud_xyzrgb.write(cloud);
	} catch (...)
	{
		LOG(LERROR) << "Error occured in processing input";
	}
}

} //: namespace DepthConventer
} //: namespace Processors
