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

const cv::Scalar CvWindow_Sink::red = cv::Scalar(0, 0, 255);
const cv::Scalar CvWindow_Sink::lime = cv::Scalar(0, 255, 0);
const cv::Scalar CvWindow_Sink::blue = cv::Scalar(255, 0, 0);
const cv::Scalar CvWindow_Sink::green = cv::Scalar(0, 128, 0);
const cv::Scalar CvWindow_Sink::black = cv::Scalar(0, 0, 0);
const cv::Scalar CvWindow_Sink::white = cv::Scalar(255, 255, 255);

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
	m_state = STATE_MOVE;
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
	
	registerHandler("Clear points", boost::bind(&CvWindow_Sink::onClearPoints, this));
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
		m_border = sl.width;
		
		cv::Mat out_img = cv::Mat::zeros(cv::Size(w, h), CV_8UC3);
		cv::Mat roi(out_img, cv::Rect(cv::Point2i(0, 0), sl));
		img_left.copyTo(roi);
		cv::Mat roi2(out_img, cv::Rect(cv::Point2i(sl.width, 0), sr));
		img_right.copyTo(roi2);
		
		for (int i = 0; i < points_l.size(); ++i) {
			cv::Point pl = points_l[i];
			cv::Point pr = points_r[i];
			cv::line(out_img, pl, pr + cv::Point(m_border, 0), white, 2);
			cv::circle(out_img, pl, 2, white, 2);
			cv::circle(out_img, pr + cv::Point(m_border, 0), 2, white, 2);
			cv::line(out_img, pl, pr + cv::Point(m_border, 0), green);
			cv::circle(out_img, pl, 2, green, 1);
			cv::circle(out_img, pr + cv::Point(m_border, 0), 2, green, 1);
			
			cv::putText(out_img, std::to_string(i), pl + cv::Point(-6, -5), cv::FONT_HERSHEY_PLAIN, 1, white);
			cv::putText(out_img, std::to_string(i) + ": " + std::to_string(pl.x - pr.x), cv::Point(5, 20+i*13), cv::FONT_HERSHEY_PLAIN, 1, white);
		}
		
		
		cv::Scalar color;
		switch (m_state) {
		case STATE_MOVE: {
			color = blue;
			cv::line(out_img, cv::Point(m_x, 0), cv::Point(m_x, h), color);
			cv::line(out_img, cv::Point(0, m_y), cv::Point(w, m_y), color); }
			break;
		case STATE_WAIT_L: {
			if (m_x >= m_border) color = red; else color = lime;
			int rx = tmp_point.x;
			int ry = tmp_point.y;
			cv::line(out_img, cv::Point(m_x, 0), cv::Point(m_x, h), color);
			cv::line(out_img, cv::Point(m_border + rx, 0), cv::Point(m_border + rx, h), color);
			cv::line(out_img, cv::Point(0, ry), cv::Point(w, ry), color); }
			break;
		case  STATE_WAIT_R: {
			if (m_x >= m_border) color = lime; else color = red;
			int lx = tmp_point.x;
			int ly = tmp_point.y;
			cv::line(out_img, cv::Point(m_x, 0), cv::Point(m_x, h), color);
			cv::line(out_img, cv::Point(lx, 0), cv::Point(lx, h), color);
			cv::line(out_img, cv::Point(0, ly), cv::Point(w, ly), color); }
			break;
		default:
			break;
		}
		
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
	m_x = x;
	m_y = y;
	
	if (event == 1) {
		cv::Point pt = cv::Point(x, y);
		switch(m_state) {
		case STATE_MOVE:
			if (x < m_border) {
				tmp_point = pt;
				m_state = STATE_WAIT_R;
			} else {
				pt.x = pt.x - m_border;
				tmp_point = pt;
				m_state = STATE_WAIT_L;
			}
			break;
		case STATE_WAIT_L:
			if (x < m_border) {
				pt.y = tmp_point.y;
				points_l.push_back(pt);
				points_r.push_back(tmp_point);
				m_state = STATE_MOVE;
			}
			break;
		case STATE_WAIT_R:
			if (x >= m_border) {
				pt.x = pt.x - m_border;
				pt.y = tmp_point.y;
				points_r.push_back(pt);
				points_l.push_back(tmp_point);
				m_state = STATE_MOVE;
			}
			break;
		}
	}
	
	//if (event == 
}


void CvWindow_Sink::onClearPoints() {
	m_state = STATE_MOVE;
	points_l.clear();
	points_r.clear();
}

}//: namespace CvWindow
}//: namespace Sinks
