/*
 * CvUndistort_Processor.cpp
 *
 *  Created on: Oct 20, 2010
 *      Author: mboryn
 */

#include "CvUndistort_Processor.hpp"

#include <opencv2/opencv.hpp>

namespace Processors {

namespace CvUndistort {

using namespace std;
using namespace cv;
using namespace boost;
using namespace Base;

CvUndistort_Processor::CvUndistort_Processor(const std::string& n) :
	Component(n),
	alpha("alpha", 0, "range")
{
	registerProperty(alpha);
}

CvUndistort_Processor::~CvUndistort_Processor()
{
}

void CvUndistort_Processor::prepareInterface() {
	registerStream("in_img_left", &in_img_left);
	registerStream("out_img_left", &out_img_left);
	registerStream("in_camera_info_left", &in_camera_info_left);
	
	registerStream("in_img_right", &in_img_right);
	registerStream("out_img_right", &out_img_right);
	registerStream("in_camera_info_right", &in_camera_info_right);

	registerHandler("onNewImage", boost::bind(&CvUndistort_Processor::onNewImage, this));
	addDependency("onNewImage", &in_img_right);
	addDependency("onNewImage", &in_camera_info_right);
	addDependency("onNewImage", &in_img_left);
	addDependency("onNewImage", &in_camera_info_left);

}

bool CvUndistort_Processor::onStart()
{
	return true;
}

bool CvUndistort_Processor::onStop()
{
	return true;
}

bool CvUndistort_Processor::onInit()
{
	interpolation = INTER_LINEAR;

	return true;
}

bool CvUndistort_Processor::onFinish()
{
	return true;
}

void CvUndistort_Processor::onNewImage()
{
	cv::Mat originalImage, undistortedImage;

	// Read input image and camera info.
	cv::Mat img_left = in_img_left.read();
	Types::CameraInfo ci_left = in_camera_info_left.read();
	cv::Mat img_right = in_img_right.read();
	Types::CameraInfo ci_right = in_camera_info_right.read();
	
	//Reprojection matix - Q
    cv::Mat Q(cv::Mat::zeros(4,4, CV_32F));
    //Rectification and projection matrices - R* and P*
    cv::Mat R1, P1, R2, P2;

    //Size of input image
    cv::Size img_size = img_left.size();

    // Camera matrices
    cv::Mat M1 = ci_left.cameraMatrix();
    cv::Mat M2 = ci_right.cameraMatrix();

    //Distortion coefficients matrices
    cv::Mat D1 = ci_left.distCoeffs();
    cv::Mat D2 = ci_right.distCoeffs();

    //Rotation and translation of second camera to the first
    cv::Mat R = ci_right.rotationMatrix();
    cv::Mat T = ci_right.translationMatrix();

	M1.convertTo(M1, CV_64F);
	M2.convertTo(M2, CV_64F);
	D1.convertTo(D1, CV_64F);
	D2.convertTo(D2, CV_64F);
	R.convertTo(R, CV_64F);
	T.convertTo(T, CV_64F);
	Q.convertTo(Q, CV_64F);
	CLOG(LDEBUG) << "M1 "<< M1 << " - " << M1.type();
	CLOG(LDEBUG) << "M2 "<< M2 << " - " << M2.type();
	CLOG(LDEBUG) << "D1 "<< D1 << " - " << D1.type();
	CLOG(LDEBUG) << "D2 "<< D2 << " - " << D2.type();
	CLOG(LDEBUG) << "R "<< R << " - " << R.type();
	CLOG(LDEBUG) << "T "<< T << " - " << T.type();
	CLOG(LDEBUG) << "Q "<< Q << " - " << Q.type();
	CLOG(LDEBUG) << "Size " << img_size.height << "x"<<img_size.width;

	cv::stereoRectify( M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, 0, 0.01 * alpha, img_size);

	CLOG(LDEBUG) << "R1 "<< R1;
	CLOG(LDEBUG) << "P1 "<< P1;
	CLOG(LDEBUG) << "R2 "<< R2;
	CLOG(LDEBUG) << "P2 "<< P2;

	cv::Mat map11, map12, map21, map22;

	CLOG(LDEBUG) << "Calculate left transformation maps";
	cv::initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
	CLOG(LDEBUG) << "Calculate right transformation maps";
	cv::initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);

	CLOG(LDEBUG) << "Rectification of images";


	cv::Mat img_left_out, img_right_out;
	cv::remap(img_left, img_left_out, map11, map12, cv::INTER_LINEAR);
	cv::remap(img_right, img_right_out, map21, map22, cv::INTER_LINEAR);
 

	out_img_left.write(img_left_out.clone());
	out_img_right.write(img_right_out.clone());
}

} // namespace CvUndistort

} // namespace Processors
