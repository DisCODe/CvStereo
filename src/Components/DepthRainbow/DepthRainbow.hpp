/*!
 * \file
 * \brief 
 * \author Dawid Kaczmarek
 */

#ifndef DEPTHRAINBOW_HPP_
#define DEPTHRAINBOW_HPP_

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
namespace DepthRainbow {


/*!
 * \class DepthRainbow
 * \brief DepthRainbow processor class.
 *
 * DepthRainbow processor.
 */
class DepthRainbow: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
    DepthRainbow(const std::string & name = "DepthRainbow");

	/*!
	 * Destructor
	 */
    virtual ~DepthRainbow();

	/*!
	 * Prepare components interface (register streams and handlers).
	 * At this point, all properties are already initialized and loaded to 
	 * values set in config file.
	 */
	void prepareInterface();

	/// Maximals sensor range (in meters).
	static const unsigned short DEPTH_RANGE = 1536;

	/// Value denoting "not a number".
	static const unsigned short NAN_VALUE = 10000;

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
    Base::DataStreamIn<cv::Mat> in_depth_xyz;

	// Output data streams
    Base::DataStreamOut<cv::Mat> out_depth_rainbow;

	// Properties
    Base::Property<bool> fixed_range;
    Base::Property<double> min_range;
    Base::Property<double> max_range;

	// Handlers
    void convertMonoToRainbow();


};

} //: namespace DepthRainbow
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("DepthRainbow", Processors::DepthRainbow::DepthRainbow)

#endif /* DEPTHRAINBOW_HPP_ */
