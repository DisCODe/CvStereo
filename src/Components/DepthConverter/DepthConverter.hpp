/*!
 * \file
 * \brief 
 * \author Dawid Kaczmarek
 */

#ifndef DEPTHCONVENTER_HPP_
#define DEPTHCONVENTER_HPP_

#include "Base/Component_Aux.hpp"
#include "Base/Component.hpp"
#include "Base/DataStream.hpp"
#include "Base/Property.hpp"
#include "Base/EventHandler2.hpp"

#include <Types/CameraInfo.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace Processors {
namespace DepthConventer {


/*!
 * \class DepthConventer
 * \brief DepthConventer processor class.
 *
 * DepthConventer processor.
 */
class DepthConventer: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
    DepthConventer(const std::string & name = "DepthConventer");

	/*!
	 * Destructor
	 */
    virtual ~DepthConventer();

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

	// Input data streams
    Base::DataStreamIn<cv::Mat, Base::DataStreamBuffer::Newest> in_rgb_stereo;
    Base::DataStreamIn<cv::Mat, Base::DataStreamBuffer::Newest> in_depth_xyz;

	// Output data streams
    Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > out_cloud_xyzrgb;

	// Properties
	// -- NONE --

	// Handlers
	void CalculateDepthMap();
};

} //: namespace DepthConventer
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("DepthConventer", Processors::DepthConventer::DepthConventer)

#endif /* DEPTHCONVENTER_HPP_ */
