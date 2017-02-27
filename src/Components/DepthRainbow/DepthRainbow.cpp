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
    registerStream("in_disparity", &in_disparity);
    registerStream("out_depth_rainbow", &out_depth_rainbow);

	// Register handlers
    registerHandler("convertMonoToRainbow", boost::bind(&DepthRainbow::convertMonoToRainbow, this));
    addDependency("convertMonoToRainbow", &in_depth_xyz);
    
    registerHandler("convertDisparityToRainbow", boost::bind(&DepthRainbow::convertDisparityToRainbow, this));
    addDependency("convertDisparityToRainbow", &in_disparity);
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

typedef struct{uchar b; uchar g; uchar r;} color;

color hsv2rgb(float hue, float sat, float val) {
	float red, grn, blu;
	float i, f, p, q, t;
	color result;
	 
	if(val==0) {
	red = 0;
	grn = 0;
	blu = 0;
	} else {
	hue/=60;
	i = floor(hue);
	f = hue-i;
	p = val*(1-sat);
	q = val*(1-(sat*f));
	t = val*(1-(sat*(1-f)));
	if (i==0) {red=val; grn=t; blu=p;}
	else if (i==1) {red=q; grn=val; blu=p;}
	else if (i==2) {red=p; grn=val; blu=t;}
	else if (i==3) {red=p; grn=q; blu=val;}
	else if (i==4) {red=t; grn=p; blu=val;}
	else if (i==5) {red=val; grn=p; blu=q;}
	}
	result.r = red * 255;
	result.g = grn * 255;
	result.b = blu * 255;
	return result;
}

double interpolate( double val, double y0, double x0, double y1, double x1 ) {
    return (val-x0)*(y1-y0)/(x1-x0) + y0;
}

double base( double val ) {
    if ( val <= -0.75 ) return 0;
    else if ( val <= -0.25 ) return interpolate( val, 0.0, -0.75, 1.0, -0.25 );
    else if ( val <= 0.25 ) return 1.0;
    else if ( val <= 0.75 ) return interpolate( val, 1.0, 0.25, 0.0, 0.75 );
    else return 0.0;
}

double red( double gray ) {
    return base( gray - 0.5 );
}
double green( double gray ) {
    return base( gray );
}
double blue( double gray ) {
    return base( gray + 0.5 );
}

void DepthRainbow::convertDisparityToRainbow() {
    cv::Mat data(in_disparity.read());
        
	cv::Mat out;
    double min, max;
	if (fixed_range) {
        min = min_range;
        max = max_range;
    } else {
        cv::minMaxLoc(data, &min, &max);
    }
    
	out.create(data.size(), CV_8UC3);
	for (int y = 0; y < out.rows; y++) {
		for (int x = 0; x < out.cols; x++) {
			color col;
			col.r = col.g = col.b = 0;
			float curDisp = data.at<float>(y, x);
			
			if (curDisp < min+0.1) {
				col.r=col.g=col.b = 0;
			} else {
				double h = (curDisp-min)/(max-min);
				if (h < 0) h = 0;
				if (h > 1) h = 1;
				//col = hsv2rgb(h*300, 1, 1);
                h = h * 2 - 1;
                col.r = red(h) * 255;
                col.g = green(h) * 255;
                col.b = blue(h) * 255;
                //col.r = col.g = col.b = 255 * h;
			}
			out.at<color>(y, x) = col;
		}
	}
    
    out_depth_rainbow.write(out);
}

void DepthRainbow::convertMonoToRainbow() {

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
    CLOG(LDEBUG) << "Converting mono to rainbow";
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
        CLOG(LINFO) << "Z coord: Min value = " << min_val << ", Max value = " << max_val;
        out_depth_rainbow.write(out);
	} catch (...)
	{
		CLOG(LERROR) << "Error occured in processing input";
	}
}

} //: namespace DepthRainbow
} //: namespace Processors
