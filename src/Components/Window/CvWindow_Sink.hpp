/*!
 * \file CvWindow_Sink.hpp
 * \brief
 * \author tkornuta
 * \date 11.03.2008
 */

#ifndef CVWINDOW_SINK_HPP_
#define CVWINDOW_SINK_HPP_

#include "Base/Component_Aux.hpp"
#include "Base/Component.hpp"
#include "Base/DataStream.hpp"
#include "Common/Logger.hpp"

#include "Base/EventHandler2.hpp"
#include "Base/Property.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

/**
 * \defgroup CvWindow CvWindow
 * \ingroup Sinks
 *
 * Creates a window and displays the image. There is option to draw additional
 * information on top of displayed image by using in_draw stream (and feeding it
 * with \ref Types::Drawable "drawable" items).
 *
 *
 *
 * \par Data streams:
 *
 * \streamin{in_img,cv::Mat}
 * Input image
 * \streamin{in_draw,Types::Drawable}
 * Things to draw on top of image.
 *
 *
 * \par Event handlers:
 *
 * \handler{onNewImage}
 * New image arrived
 *
 *
 * \par Properties:
 *
 * \prop{title,string,"video"}
 * Window caption
 *
 *
 * \see http://opencv.willowgarage.com/documentation/cpp/user_interface.html#namedWindow
 * \see http://opencv.willowgarage.com/documentation/cpp/user_interface.html#cv-imshow
 *
 *
 * \par Task configuration template:
 *
 * \code
 * <Components>
 *   <Window type="CvWindow" thread="thread_name" group="group_name">
 *     <title>video</title>
 *   </Window>
 * </Components>
 *
 * <Events>
 *   <Event source="Component.Event" destination="Window.onNewImage"/>
 * </Events>
 *
 * <DataStreams>
 *   <Window>
 *     <in_img type="in" group="connection_name"/>
 *   </Window>
 * </DataStreams>
 * \endcode
 *
 * @{
 *
 * @}
 */

namespace Types {
class Drawable;
}

namespace Sinks {
namespace CvWindow {

using namespace cv;

class CvWindow_Sink;

struct MouseCallbackInfo {
	CvWindow_Sink * parent;
	
	MouseCallbackInfo(CvWindow_Sink * p = NULL) : parent(p) {}
};

/*!
 * \class CvWindow_Sink
 * \brief Creates a window and displays the image
 */
class CvWindow_Sink: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	CvWindow_Sink(const std::string & name = "");

	/*!
	 * Destructor
	 */
	virtual ~CvWindow_Sink();

	virtual void prepareInterface();

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
	 * Retrieves data from device.
	 */
	bool onStep();

	/*!
	 * Start component
	 */
	bool onStart();

	/*!
	 * Stop component
	 */
	bool onStop();

	/*!
	 * Event handler function.
	 */
	void onNewImage();

	/*!
	 * Event handler function.
	 */
	void onRefresh();

	Base::DataStreamIn<Mat, Base::DataStreamBuffer::Newest, Base::Synchronization::Mutex>  in_img_left;
	Base::DataStreamIn<Mat, Base::DataStreamBuffer::Newest, Base::Synchronization::Mutex>  in_img_right;

	/// Image to be drawn.
	cv::Mat img_left;
	cv::Mat img_right;

	Base::Property<std::string> title;

	Base::Property<std::string> filename;
	Base::Property<std::string> dir;
	
	
	MouseCallbackInfo * callback_info;
	static void onMouseStatic(int event, int x, int y, int flags, void * userdata);
	
	void onMouse(int event, int x, int y, int flags);
	
	Base::Property<bool> mouse_tracking;
	
	int m_x, m_y;
	
	std::vector<cv::Point> points_l, points_r;
};

}//: namespace CvWindow
}//: namespace Sinks


/*
 * Register processor component.
 */
REGISTER_COMPONENT("CvWindow", Sinks::CvWindow::CvWindow_Sink)

#endif /* CVWINDOW_SINK_HPP_ */
