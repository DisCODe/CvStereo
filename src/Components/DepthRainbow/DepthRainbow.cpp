/*!
 * \file
 * \brief
 * \author Dawid Kaczmarek
 */

#include <memory>
#include <string>

#include "DepthRainbow.hpp"
#include "Common/Logger.hpp"
#include <Types/MatrixTranslator.hpp>

#include <boost/bind.hpp>
#include <boost/format.hpp>

namespace Processors {
namespace DepthRainbow {

DepthRainbow::DepthRainbow(const std::string & name) :
        Base::Component(name),
    fixed_range("fixed_range", false),
    min_range("min_range", double(-1.0)),
    max_range("max_range", double(1.0))
{
    registerProperty(fixed_range);
    registerProperty(min_range);
    registerProperty(max_range);
}

DepthRainbow::~DepthRainbow() {
}

void DepthRainbow::prepareInterface() {
	// Register data streams, events and event handlers HERE!
    registerStream("in_depth_xyz", &in_depth_xyz);
    registerStream("out_depth_rainbow", &out_depth_rainbow);

	// Register handlers
    registerHandler("convertMonoToRainbow", boost::bind(&DepthRainbow::convertMonoToRainbow, this));
    addDependency("convertMonoToRainbow", &in_depth_xyz);
}

bool DepthRainbow::onInit() {
	return true;
}

bool DepthRainbow::onFinish() {
	return true;
}

bool DepthRainbow::onStop() {
	return true;
}

bool DepthRainbow::onStart() {
	return true;
}

typedef struct{uchar r; uchar g; uchar b;} color;

void DepthRainbow::convertMonoToRainbow() {
    unsigned short DEPTH_RANGE = 1536;
    unsigned short NAN_VALUE = 10000;

    cv::Mat data(in_depth_xyz.read());
    cv::Mat depth;
    cv::Mat depth_8bit;
    cv::Mat out;
    float max_val = 0;
    float min_val = NAN_VALUE;
    float delta;
    depth.create(data.size(), CV_32F);
    for (int y = 0; y < data.rows; y++) {
        for (int x = 0; x < data.cols; x++) {
            cv::Vec3f point = data.at<cv::Vec3f>(y, x);
            if (max_val < point[2] & point[2] != NAN_VALUE) max_val = point[2];
            if (min_val > point[2]) min_val = point[2];
            depth.at<float>(y, x) = point[2];
        }
    }
    if (fixed_range)
    {
        min_val = min_range;
        max_val = max_range;
    }
    depth.convertTo(depth_8bit, CV_8U);
    delta = (max_val - min_val) / DEPTH_RANGE;
    LOG(LDEBUG) << "Converting mono to rainbow";
    try {
        out.create(data.size(), CV_8UC3);
        for (int y = 0; y < out.rows; y++) {
            for (int x = 0; x < out.cols; x++) {
                color col;
                col.r = col.g = col.b = 0;
                float z_val = depth.at<float>(y, x);
                z_val = ((z_val - min_val) / delta);
                if (z_val >= DEPTH_RANGE)
                {
                    col.r = col.g = col.b = 0;
                } else {
                    unsigned short curDepth = (unsigned short) z_val;
                    unsigned short lb = curDepth & 0xff;
                    unsigned short rainbowPart = curDepth>>8;

                    switch (rainbowPart) {
                    case 0:
                        col.b = 255;
                        col.g = 255-lb;
                        col.r = 255-lb;
                        break;
                    case 1:
                        col.b = 255;
                        col.g = lb;
                        col.r = 0;
                        break;
                    case 2:
                        col.b = 255-lb;
                        col.g = 255;
                        col.r = 0;
                        break;
                    case 3:
                        col.b = 0;
                        col.g = 255;
                        col.r = lb;
                        break;
                    case 4:
                        col.b = 0;
                        col.g = 255-lb;
                        col.r = 255;
                        break;
                    case 5:
                        col.b = 0;
                        col.g = 0;
                        col.r = 255-lb;
                        break;
                    default:
                        col.r = col.g = col.b = 0;
                        break;
                    }
                }
                out.at<color>(y, x) = col;
            }
        }
        LOG(LINFO) << "Z coord: Min value = " << min_val << ", Max value = " << max_val;
        out_depth_rainbow.write(out);
	} catch (...)
	{
		LOG(LERROR) << "Error occured in processing input";
	}
}

} //: namespace DepthRainbow
} //: namespace Processors
