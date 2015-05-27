/*!
 * \file
 * \brief
 * \author Łukasz Żmuda
 */

#include <memory>
#include <string>

#include "DepthTransform.hpp"
#include "Common/Logger.hpp"
#include <Types/MatrixTranslator.hpp>
#include "Property.hpp"

#include <boost/bind.hpp>
#include <boost/format.hpp>

namespace Processors {
namespace DepthTransform {
  
using Types::HomogMatrix;

DepthTransform::DepthTransform(const std::string & name) :
	Base::Component(name),
	prop_inverse("inverse", false)
{
    registerProperty(prop_inverse);

}

DepthTransform::~DepthTransform() {

}

void DepthTransform::prepareInterface() {

	// Register data streams, events and event handlers HERE!
	registerStream("in_homogMatrix", &in_homogMatrix);
	registerStream("in_image_xyz", &in_image_xyz);
	registerStream("out_image_xyz", &out_image_xyz);

	// Register handlers
	registerHandler("DepthTransformation", boost::bind(&DepthTransform::DepthTransformation, this));
	addDependency("DepthTransformation", &in_image_xyz);
	addDependency("DepthTransformation", &in_homogMatrix);

}

bool DepthTransform::onInit() {

	return true;
}

bool DepthTransform::onFinish() {
	return true;
}

bool DepthTransform::onStop() {
	return true;
}

bool DepthTransform::onStart() {
	return true;
}

void DepthTransform::DepthTransformation() {
	try{
	  
	cv::Mat img = in_image_xyz.read();
	HomogMatrix tmp_hm = in_homogMatrix.read();
	HomogMatrix hm;

	CLOG(LDEBUG) << "Input homogenous matrix:\n" << tmp_hm;

	// Check inversion property.
	if (prop_inverse){
		hm.matrix() = tmp_hm.matrix().inverse();
	}
	else
		hm = tmp_hm;

	// Temporary point.
	Eigen::Vector4d pt;

	CLOG(LINFO) << "Using homogenous matrix:\n" << hm;

	// check, if image has proper number of channels
	if (img.channels() != 3) {
		CLOG(LERROR) << "DepthTransformation: Wrong number of channels";
		return;
	}
	
	// check image depth, allowed is only 32F and 64F
	int img_type = img.depth();
	if ( (img_type != CV_32F) && (img_type != CV_64F) ) {
		CLOG(LERROR) << "DepthTransformation: Wrong depth";
		return;
	}
	
	// clone image - operation will change its contents
	img = img.clone();
	
	// Check size.	
	int rows = img.rows;
	int cols = img.cols;
	
	if (img.isContinuous()) {
		cols *= rows;
		rows = 1;
	}
	
	// float variant
	if (img_type == CV_32F) {
		CLOG(LINFO) << "Transflorming CV_32F";
		int i,j;
		float* p;

		// iterate through all image pixels
		for( i = 0; i < rows; ++i) {

			p = img.ptr<float>(i);
			for ( j = 0; j < cols; ++j) {
				// Filter invalid numbers.
				if(p[3*j + 2] > 300)
					continue;

				// read point coordinates 
				pt(0) = p[3*j];
				pt(1) = p[3*j + 1];
				pt(2) = p[3*j + 2];
				pt(3) = 1;

				// transform point
				pt = hm * pt;

				// write back result
				p[3*j]   = pt(0);
				p[3*j+1] = pt(1);
				p[3*j+2] = pt(2);
				
			}//: for
		}//: for
		
	} else { // double variant
		CLOG(LINFO) << "Transflorming CV_64F";
		int i,j;
		double* p;
		for( i = 0; i < rows; ++i) {
			p = img.ptr<double>(i);
			for ( j = 0; j < cols; ++j) {
				// Filter invalid numbers.
				if(p[3*j + 2] > 300)
					continue;

				// read point coordinates 
				pt(0) = p[3*j];
				pt(1) = p[3*j + 1];
				pt(2) = p[3*j + 2];
				pt(3) = 1;

				// transform point
				pt = hm * pt;

				// write back result
				p[3*j]   = pt(0);
				p[3*j+1] = pt(1);
				p[3*j+2] = pt(2);
			}//: for
		}//: for
	}//: else
	
	out_image_xyz.write(img);

	} catch (...)
	{
		LOG(LERROR) << "DepthTransformation:Error occured in processing input";
	}
}


} //: namespace DepthTransform
} //: namespace Processors
