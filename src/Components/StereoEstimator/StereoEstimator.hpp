/*!
 * \file
 * \brief 
 * \author Dawid Kaczmarek
 */

#ifndef STEREOESTIMATOR_HPP_
#define STEREOESTIMATOR_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <Types/CameraInfo.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace Processors {
namespace StereoEstimator {

static const string STEREO_BM = "STEREO_BM";
static const string STEREO_SGBM = "STEREO_SGBM";
static const string STEREO_HH = "STEREO_HH";
//static const string STEREO_VAR = "STEREO_VAR";

/*!
 * \class StereoEstimator
 * \brief StereoEstimator processor class.
 *
 * StereoEstimator processor.
 */
class StereoEstimator: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
    StereoEstimator(const std::string & name = "StereoEstimator");

	/*!
	 * Destructor
	 */
    virtual ~StereoEstimator();

	/*!
	 * Prepare components interface (register streams and handlers).
	 * At this point, all properties are already initialized and loaded to 
	 * values set in config file.
	 */
	void prepareInterface();

protected:

	/*!
	 * Connects source to given device.
	 */
	bool onInit();

	/*!
	 * Disconnect source from device, closes streams, etc.
	 */
	bool onFinish();

	/*!
	 * Start component
	 */
	bool onStart();

	/*!
	 * Stop component
	 */
	bool onStop();

    /*!
     * Generate Q matrix for 2 projection matrices
     */
    void generateQ(const cv::Mat& leftPMatrix, const cv::Mat& rightPMatrix, cv::Mat& Q);
    void generateQ(const cv::Mat& leftCMatrix, const cv::Mat& rightCMatrix, const cv::Mat& translateMatrix, cv::Mat& Q);

	// Input data streams
    Base::DataStreamIn<cv::Mat, Base::DataStreamBuffer::Newest, Base::Synchronization::Mutex> l_in_img;
    Base::DataStreamIn<cv::Mat, Base::DataStreamBuffer::Newest, Base::Synchronization::Mutex> r_in_img;
	Base::DataStreamIn<Types::CameraInfo> l_in_cam_info;
	Base::DataStreamIn<Types::CameraInfo> r_in_cam_info;

	// Output data streams
    Base::DataStreamOut<cv::Mat> out_rgb_stereo;
	Base::DataStreamOut<cv::Mat> out_depth_map;
    Base::DataStreamOut<cv::Mat> out_depth_xyz;


	// Properties
	Base::Property<string> algorythm_type;
    Base::Property<int> numberOfDisparities;
    Base::Property<int> SADWindowSize;
    Base::Property<int> preFilterCap;
    Base::Property<int> minDisparity;
    Base::Property<int> disp12MaxDiff;
    Base::Property<int> speckleRange;
    Base::Property<int> speckleWindowSize;
    Base::Property<int> uniquenessRatio;
    Base::Property<int> textureThreshold;
    Base::Property<bool> prop_rectify;


	// Handlers
	Base::EventHandler2 h_CalculateDepthMap;

	cv::StereoSGBM* sgbm;
	cv::StereoBM* bm;

    cv::Mat in_left_image;
    cv::Mat in_right_image;
    Types::CameraInfo in_left_cam_info;
    Types::CameraInfo in_right_cam_info;

	// Handlers
	void CalculateDepthMap();
};

} //: namespace StereoEstimator
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("StereoEstimator", Processors::StereoEstimator::StereoEstimator)

#endif /* STEREOESTIMATOR_HPP_ */
