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
	
	m_x = m_y = 0;
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
			img_right = in_img_right.read().clone();
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
		int w = sl.width + sr.width;
		int h = std::max(sl.height, sr.height);
		
		cv::Mat out_img = cv::Mat::zeros(cv::Size(w, h), CV_8UC3);
		cv::Mat roi(out_img, cv::Rect(cv::Point2i(0, 0), sl));
		img_left.copyTo(roi);
		cv::Mat roi2(out_img, cv::Rect(cv::Point2i(sl.width, 0), sr));
		img_right.copyTo(roi2);
		
		for (int i = 0; i < points_l.size(); ++i) {
			cv::circle(out_img, points_l[i], 2, cv::Scalar(64, 192, 64), 2);
		}
		
		for (int i = 0; i < points_r.size(); ++i) {
			cv::circle(out_img, points_r[i], 2, cv::Scalar(64, 192, 64), 2);
			cv::line(out_img, points_l[i], points_r[i], cv::Scalar(64, 192, 64));
		}
		
		cv::line(out_img, cv::Point(m_x, 0), cv::Point(m_x, h), cv::Scalar(0, 0, 255));
		cv::line(out_img, cv::Point(0, m_y), cv::Point(w, m_y), cv::Scalar(0, 0, 255));
		
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
	CLOG(LNOTICE) << "Click in at " << x << "," << y << " [" << event << "]";
	m_x = x;
	m_y = y;
	
	if (event == 1) {
		if (points_l.size() == points_r.size())
			points_l.push_back(cv::Point(x, y));
		else
			points_r.push_back(cv::Point(x, y));
	}
}

}//: namespace CvWindow
}//: namespace Sinks
