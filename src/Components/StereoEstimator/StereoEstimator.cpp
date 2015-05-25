/*!
 * \file
 * \brief
 * \author Dawid Kaczmarek
 */

#include <memory>
#include <string>

#include "StereoEstimator.hpp"
#include "Common/Logger.hpp"
#include <Types/MatrixTranslator.hpp>

#include <boost/bind.hpp>
#include <boost/format.hpp>

namespace Processors {
namespace StereoEstimator {

StereoEstimator::StereoEstimator(const std::string & name) :
		Base::Component(name),
        numberOfDisparities("numberOfDisparities", int(0)),
        SADWindowSize("SADWindowSize (MUST BE ODD >= 1)", int(5)),
        preFilterCap("preFilterCap", int(63)),
        minDisparity("minDisparity", int(0)),
        disp12MaxDiff("disp12MaxDiff", int(1)),
        speckleRange("speckleRange", int(32)),
        speckleWindowSize("speckleWindowSize", int(100)),
        uniquenessRatio("uniquenessRatio", int(10)),
        textureThreshold("textureThreshold", int(10)),
        algorythm_type("algorythm_type", STEREO_SGBM, "combo"),
        prop_rectify("perform_rectification", true)
{
    bm = NULL;
    sgbm = NULL;
    algorythm_type.addConstraint(STEREO_BM);
    algorythm_type.addConstraint(STEREO_SGBM);
    algorythm_type.addConstraint(STEREO_HH);
    registerProperty(algorythm_type);
    numberOfDisparities.addConstraint("1");
    numberOfDisparities.addConstraint("9999");
    registerProperty(numberOfDisparities);
    SADWindowSize.addConstraint("1");
    SADWindowSize.addConstraint("9999");
    registerProperty(SADWindowSize);
    preFilterCap.addConstraint("1");
    preFilterCap.addConstraint("9999");
    registerProperty(preFilterCap);
    minDisparity.addConstraint("0");
    minDisparity.addConstraint("9999");
    registerProperty(minDisparity);
    disp12MaxDiff.addConstraint("0");
    disp12MaxDiff.addConstraint("9999");
    registerProperty(disp12MaxDiff);
    speckleRange.addConstraint("0");
    speckleRange.addConstraint("9999");
    registerProperty(speckleRange);
    speckleWindowSize.addConstraint("0");
    speckleWindowSize.addConstraint("9999");
    registerProperty(speckleWindowSize);
    uniquenessRatio.addConstraint("0");
    uniquenessRatio.addConstraint("100");
    registerProperty(uniquenessRatio);
    textureThreshold.addConstraint("0");
    textureThreshold.addConstraint("9999");
    registerProperty(textureThreshold);
    registerProperty(prop_rectify);
}

StereoEstimator::~StereoEstimator() {
	if (sgbm != 0 ) delete sgbm;
	if (bm != 0 ) delete bm;
}

void StereoEstimator::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("l_in_img", &l_in_img);
	registerStream("r_in_img", &r_in_img);
	registerStream("l_cam_info", &l_in_cam_info);
	registerStream("r_cam_info", &r_in_cam_info);
    registerStream("out_rgb_stereo", &out_rgb_stereo);
    registerStream("out_depth_map", &out_depth_map);
    registerStream("out_depth_xyz", &out_depth_xyz);

	// Register handlers
	registerHandler("CalculateDepthMap", boost::bind(&StereoEstimator::CalculateDepthMap, this));
	addDependency("CalculateDepthMap", &l_in_img);
	addDependency("CalculateDepthMap", &r_in_img);
}

bool StereoEstimator::onInit() {
    sgbm = new cv::StereoSGBM();
    bm = new cv::StereoBM();
	return true;
}

bool StereoEstimator::onFinish() {
	return true;
}

bool StereoEstimator::onStop() {
	return true;
}

bool StereoEstimator::onStart() {
	return true;
}

void StereoEstimator::CalculateDepthMap() {
    CLOG(LINFO) << "StereoEstimator::CalculateDepthMap";
    CLOG(LDEBUG) << "Get images from Data Streams";
    {
        if (l_in_img.fresh())
        {
            LOG(LINFO) << "Read fresh left image from Data Streams";
            in_left_image = l_in_img.read();
        }
        if (r_in_img.fresh())
        {
            LOG(LINFO) << "Read fresh right image from Data Streams";
            in_right_image = r_in_img.read();
        }
        if (l_in_cam_info.fresh())
        {
            LOG(LINFO) << "Read fresh left CameraInfo from Data Streams";
            in_left_cam_info = l_in_cam_info.read();
        }
        if (r_in_cam_info.fresh())
        {
            LOG(LINFO) << "Read fresh right CameraInfo from Data Streams";
            in_right_cam_info = r_in_cam_info.read();
        }
    }
    //Source images
    cv::Mat& oLeftImage(in_left_image);
    cv::Mat& oRightImage(in_right_image);
    //Rectified images
    cv::Mat oLeftRectified;
    cv::Mat oRightRectified;
    //Camera information
    Types::CameraInfo& oLeftCamInfo(in_left_cam_info);
    Types::CameraInfo& oRightCamInfo(in_right_cam_info);

    //Convert to monochrome if BM algorithm is chosen
    if( algorythm_type == STEREO_BM ){
    	cv::Mat LgreyMat, RgreyMat;
    	cv::cvtColor(oLeftImage, LgreyMat, CV_BGR2GRAY);
    	cv::cvtColor(oRightImage, RgreyMat, CV_BGR2GRAY);
    	oLeftImage=LgreyMat;
    	oRightImage=RgreyMat;
    }

	try {


    CLOG(LDEBUG) << "Create bunch of fresh cv::Mat objects";
    //Region of intrest - valid rectified image space
    cv::Rect roi1, roi2;
    //Reprojection matix - Q
    cv::Mat Q(cv::Mat::zeros(4,4, CV_64F));
    //Rectification and projection matrices - R* and P*
    cv::Mat R1, P1, R2, P2;

    //Size of input image
    cv::Size img_size = oLeftImage.size();

    // Camera matrices
    cv::Mat M1 = oLeftCamInfo.cameraMatrix();
    cv::Mat M2 = oRightCamInfo.cameraMatrix();

    //Distortion coefficients matrices
    cv::Mat D1 = oLeftCamInfo.distCoeffs();
    cv::Mat D2 = oRightCamInfo.distCoeffs();

    //Rotation and translation of second camera to the first
    cv::Mat R = oRightCamInfo.rotationMatrix();
    cv::Mat T = oRightCamInfo.translationMatrix();

    if(prop_rectify)
    {
        CLOG(LDEBUG) << "M1 "<< M1;
        CLOG(LDEBUG) << "M2 "<< M2;
        CLOG(LDEBUG) << "D1 "<< D1;
        CLOG(LDEBUG) << "D2 "<< D2;
        CLOG(LDEBUG) << "Size " << img_size.height << "x"<<img_size.width;

        cv::stereoRectify( M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, 0, img_size, &roi1, &roi2 );

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

        cv::remap(oLeftImage, oLeftRectified, map11, map12, cv::INTER_LINEAR);
        cv::remap(oRightImage, oRightRectified, map21, map22, cv::INTER_LINEAR);
    } else
    {
        generateQ(M1,M2,T,Q);
        oLeftRectified = oLeftImage;
        oRightRectified = oRightImage;
    }
    CLOG(LDEBUG) << "#ofDisparities: " << numberOfDisparities;

    numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width/8) + 15) & -16;

    bm->state->roi1 = roi1;
    bm->state->roi2 = roi2;
    bm->state->preFilterCap = preFilterCap;
    bm->state->SADWindowSize = SADWindowSize;
    bm->state->minDisparity = minDisparity;
    bm->state->numberOfDisparities = numberOfDisparities;
    bm->state->textureThreshold = textureThreshold;
    bm->state->uniquenessRatio = uniquenessRatio;
    bm->state->speckleWindowSize = speckleWindowSize;
    bm->state->speckleRange = speckleRange;
    bm->state->disp12MaxDiff = disp12MaxDiff;


    int cn = oLeftImage.channels();

    sgbm->preFilterCap = preFilterCap;
    int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
    sgbm->SADWindowSize = sgbmWinSize;
    sgbm->P1 = 8*cn*sgbmWinSize*sgbmWinSize;
    sgbm->P2 = 32*cn*sgbmWinSize*sgbmWinSize;
    sgbm->minDisparity = minDisparity;
    sgbm->numberOfDisparities = numberOfDisparities;
    sgbm->uniquenessRatio = uniquenessRatio;
    sgbm->speckleWindowSize = speckleWindowSize;
    sgbm->speckleRange = speckleRange;
    sgbm->disp12MaxDiff = disp12MaxDiff;
    sgbm->fullDP = ( algorythm_type == STEREO_HH );

    cv::Mat disp, disp8, disp32f;

    CLOG(LDEBUG) << "Calculating disparity";
    int64 t = cv::getTickCount();
    if( algorythm_type == STEREO_BM )
        bm->operator ()(oLeftRectified, oRightRectified, disp);
    else if( algorythm_type == STEREO_SGBM || algorythm_type == STEREO_HH )
        sgbm->operator ()(oLeftRectified, oRightRectified, disp);
    t = cv::getTickCount() - t;
    float time = t * 1000/cv::getTickFrequency();
    CLOG(LINFO) << boost::format("Calculating disparity : Time elapsed: %1%ms") % time;

    CLOG(LDEBUG) << "Converting disparity to depth image";
    disp.convertTo(disp8, CV_8U, 255/(numberOfDisparities*16.));

    disp.convertTo(disp32f, CV_32F, 1./16);

    CLOG(LDEBUG) << "Generating depth point cloud";
    cv::Mat xyz;
    reprojectImageTo3D(disp32f, xyz, Q, true);

    CLOG(LDEBUG) << "Writing to data stream";
    cv::Rect roi2_copy(roi2);
    roi2_copy.x += minDisparity;
    cv::Rect mergedRoi = roi1 & roi2_copy;
    if (prop_rectify)
    {
        out_depth_map.write(disp8);
        out_rgb_stereo.write(oLeftRectified);
        out_depth_xyz.write(xyz);
    } else {
        out_depth_map.write(disp8);
        out_rgb_stereo.write(oLeftRectified);
        out_depth_xyz.write(xyz);
    }
    } catch (...)
	{
		CLOG(LERROR) << "Error occured in processing input";
	}
}

void StereoEstimator::generateQ(const cv::Mat& leftPMatrix, const cv::Mat& rightPMatrix, cv::Mat& Q) {
    double rFx = rightPMatrix.at<double>(0,0);
    double Tx = rightPMatrix.at<double>(0,3) / -rFx;
    double rCx = rightPMatrix.at<double>(0,2);
    double rCy = rightPMatrix.at<double>(1,2);
    double lCx = leftPMatrix.at<double>(0,2);
    Q = cv::Mat::zeros(4,4, CV_64F);
    Q.data[0,0] = 1.0;
    Q.data[1,1] = 1.0;
    Q.data[3,2] = -1.0 / Tx;
    Q.data[0,3] = -rCx;
    Q.data[1,3] = -rCy;
    Q.data[2,3] = rFx;
    Q.data[3,3] = (rCx - lCx) / Tx;
}

void StereoEstimator::generateQ(const cv::Mat& leftCMatrix, const cv::Mat& rightCMatrix, const cv::Mat& translateMatrix, cv::Mat& Q) {
    LOG(LINFO) << "Generating Q";
    //LOG(LINFO) << "Left Camera" << leftCMatrix;
    //LOG(LINFO) << "Right Camera" << rightCMatrix;
    //LOG(LINFO) << "Translation" << translateMatrix;
    double rFx = rightCMatrix.at<double>(0,0);
    double Tx = translateMatrix.at<double>(0,0) / -rFx;
    double rCx = rightCMatrix.at<double>(0,2);
    double rCy = rightCMatrix.at<double>(1,2);
    double lCx = leftCMatrix.at<double>(0,2);

    Q.at<double>(0,0) = 1.0;
    Q.at<double>(1,1) = 1.0;
    Q.at<double>(3,2) = -1.0 / Tx;
    Q.at<double>(0,3) = -rCx;
    Q.at<double>(1,3) = -rCy;
    Q.at<double>(2,3) = rFx;
    Q.at<double>(3,3) = (rCx - lCx) / Tx;
    LOG(LINFO) << "Q " << Q;
}

} //: namespace StereoEstimator
} //: namespace Processors
