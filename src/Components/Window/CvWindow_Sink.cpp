/*!
 * \file CvWindow_Sink.cpp
 * \brief
 * \author mstefanc
 * \date 2010-05-15
 */

#include <memory>
#include <string>
#include <iostream>
#include <stdexcept>
#include <ctime>

#include "CvWindow_Sink.hpp"
#include "Common/Logger.hpp"
#include "Types/Drawable.hpp"
#include "Types/DrawableContainer.hpp"

#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>

namespace Sinks {
namespace CvWindow {

CvWindow_Sink::CvWindow_Sink(const std::string & name) :
	Base::Component(name), 
			title("title", name), 
			dir("save.directory", std::string("./")),
			filename("save.filename", name),
			mouse_tracking("mouse.tracking", true)
{
	CLOG(LTRACE) << "Hello CvWindow_Sink\n";

	registerProperty(title);

	registerProperty(filename);
	registerProperty(dir);

	registerProperty(mouse_tracking);
	
}

CvWindow_Sink::~CvWindow_Sink() {
	CLOG(LTRACE) << "Good bye CvWindow_Sink";
}

void CvWindow_Sink::prepareInterface() {
	CLOG(LTRACE) << "CvWindow_Sink::configure";

	registerHandler("onRefresh", boost::bind(&CvWindow_Sink::onRefresh, this));
	addDependency("onRefresh", NULL);

	registerStream("in_img_left", &in_img_left);
	registerStream("in_img_right", &in_img_right);

	registerHandler(std::string("onNewImage"), boost::bind(&CvWindow_Sink::onNewImage, this));
	addDependency(std::string("onNewImage"), &in_img_left);
	addDependency(std::string("onNewImage"), &in_img_right);
}

bool CvWindow_Sink::onInit() {
	CLOG(LTRACE) << "CvWindow_Sink::initialize\n";

	cv::namedWindow(title());
	
	// mouse callbacks
	MouseCallbackInfo * cbi = new MouseCallbackInfo(this);
	callback_info = cbi;
	
	cv::setMouseCallback(title(), &CvWindow_Sink::onMouseStatic, cbi);
	
	CLOG(LTRACE) << "CvWindow_Sink::initialize done\n";
	return true;
}

bool CvWindow_Sink::onFinish() {
	CLOG(LTRACE) << "CvWindow_Sink::finish\n";

#if CV_MAJOR_VERSION<2 || CV_MINOR_VERSION<2
	cv::destroyWindow(title());
#endif

	return true;
}

bool CvWindow_Sink::onStep() {
	return true;
}

bool CvWindow_Sink::onStop() {
	CLOG(LTRACE) << name() << "::onStop";
	return true;
}

bool CvWindow_Sink::onStart() {
	CLOG(LTRACE) << name() << "::onStart";
	return true;
}

void CvWindow_Sink::onNewImage() {
	CLOG(LTRACE) << name() << "::onNewImage";

	try {
		if (!in_img_left.empty()) {
			img_left = in_img_left.read().clone();
			if (img_left.channels() == 1) {
				cv::cvtColor(img_left, img_left, CV_GRAY2BGR);
			}
		}
		
		if (!in_img_right.empty()) {
			img_left = in_img_right.read().clone();
			if (img_right.channels() == 1) {
				cv::cvtColor(img_right, img_right, CV_GRAY2BGR);
			}
		}
	} catch (std::exception &ex) {
		CLOG(LERROR) << "CvWindow::onNewImage failed: " << ex.what() << "";
	}
}

void CvWindow_Sink::onRefresh() {
	CLOG(LTRACE) << "CvWindow_Sink::step";

	try {

		if (img_left.empty()) {
			CLOG(LWARNING) << name() << ": left image empty";
			return;
		}
		
		if (img_right.empty()) {
			CLOG(LWARNING) << name() << ": right image empty";
			return;
		}
		
		cv::Size sl = img_left.size();
		cv::Size sr = img_right.size();
		int ll = 0;
		int lr = sl.width;
		int rl = lr+1;
		int rr = rl + sr.width;
		int lt = 0;
		int lb = sl.height;
		int rt = 0;
		int rb = sr.height;
		
		cv::Mat out_img = cv::Mat::zeros(cv::Size(sl.width+sr.width, std::max(sl.height, sr.height)), CV_8UC3);
		img_left.copyTo(out_img.colRange(ll, lr).rowRange(lt, lb));
		img_right.copyTo(out_img.colRange(rl, rr).rowRange(rt, rb));
		
		
		// Refresh image.
		imshow(title(), out_img);
		waitKey(2);
	
	} catch (...) {
		CLOG(LERROR) << "CvWindow::onStep failed\n";
	}
}

void CvWindow_Sink::onMouseStatic(int event, int x, int y, int flags, void * userdata) {
	MouseCallbackInfo * cbi = (MouseCallbackInfo*)userdata;
	cbi->parent->onMouse(event, x, y, flags);
}
	
void CvWindow_Sink::onMouse(int event, int x, int y, int flags) {
	if (event != 0 || mouse_tracking) {
		//CLOG(LNOTICE) << "Click in " << titles[window] << " at " << x << "," << y << " [" << event << "]";
		//out_point[window]->write(cv::Point(x, y));
	}
}

}//: namespace CvWindow
}//: namespace Sinks
