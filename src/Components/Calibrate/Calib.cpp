/*!
 * \file
 * \brief
 * \author Tomasz Kornuta [tkornuta@ia.pw.edu.pl]
 */

#include <memory>
#include <string>

#include "Calib.hpp"
#include "Common/Logger.hpp"
#include "Types/MatrixTranslator.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace Calib {

Calib::Calib(const std::string & name) :
		Base::Component(name),
		continuous("continuous", false),
		two_step("two_Step", false)
{
	addObject3D = false;
	// Register properties.
	registerProperty(continuous);
	registerProperty(two_step);
}

Calib::~Calib() {
}

void Calib::prepareInterface() {
	// Register data streams
	registerStream("in_object3d_left", &in_object3D_left);
	registerStream("in_object3d_right", &in_object3D_right);
	registerStream("in_camera_info", &in_camerainfo);
	registerStream("out_camera_info_left", &out_camerainfo_left);
	registerStream("out_camera_info_right", &out_camerainfo_right);

	// Register handler processing the object3D.
	registerHandler("process_object3D", boost::bind(&Calib::process_object3D, this));
	addDependency("process_object3D", &in_object3D_left);
	addDependency("process_object3D", &in_object3D_right);

	// Register handler performing the calibration.
	registerHandler("perform_calibration", boost::bind(&Calib::perform_calibration, this));

	// Register handler setting the flag for acquisition of a single object3D.
	registerHandler("add_object3D", boost::bind(&Calib::add_object3D, this));

	// Register handler realizing the clearance of the whole dataset.
	registerHandler("clear_dataset", boost::bind(&Calib::clear_dataset, this));
}

bool Calib::onInit() {

	return true;
}

bool Calib::onFinish() {
	return true;
}

bool Calib::onStop() {
	return true;
}

bool Calib::onStart() {
	return true;
}

void Calib::process_object3D() {
	CLOG(LTRACE) << "Calib::process_chessboard";
	// Check component working mode.
	if (addObject3D || continuous) {
		// Reset flag.
		addObject3D = false;

		// Retrieve chessboard from the inputstream.
		Types::Objects3D::Object3D object_left = in_object3D_left.read();
		Types::Objects3D::Object3D object_right = in_object3D_right.read();
		Types::CameraInfo camera_info = in_camerainfo.read();

		imageSize = camera_info.size();

		// Add image points.
		imagePoints_left.push_back(object_left.getImagePoints());
		imagePoints_right.push_back(object_right.getImagePoints());

		// Add object points.
		objectPoints_left.push_back(object_left.getModelPoints());
		objectPoints_right.push_back(object_right.getModelPoints());

		CLOG(LINFO) << "Registered new set of points";
	}
}

void Calib::add_object3D()
{
	addObject3D = true;
	CLOG(LINFO) << "Next set of points will be registered";
}

void Calib::clear_dataset()
{
	imagePoints_left.clear();
	objectPoints_left.clear();
	imagePoints_right.clear();
	objectPoints_right.clear();
	CLOG(LINFO) << "Dataset cleared";
}

void Calib::perform_calibration()
{
    CLOG(LINFO) << "Calib::perform_calibration()";

    if(imagePoints_left.size() > 0) {
			// The 3x3 camera matrix containing focal lengths fx,fy and displacement of the center of coordinates cx,cy.
			cv::Mat cameraMatrix_left = cv::Mat::eye(3, 3, CV_32F);
			cv::Mat cameraMatrix_right = cv::Mat::eye(3, 3, CV_32F);
			// Set two focal lengths in intrinsic matrix to have a ratio of 1.
			cameraMatrix_left.at<double>(0,0) = 1.0f;
			cameraMatrix_left.at<double>(1,1) = 1.0f;
			cameraMatrix_right.at<double>(0,0) = 1.0f;
			cameraMatrix_right.at<double>(1,1) = 1.0f;

			// Vector with distortion coefficients k_1, k_2, p_1, p_2, k_3.
			cv::Mat distCoeffs_left = cv::Mat::zeros(8, 1, CV_32F);
			cv::Mat distCoeffs_right = cv::Mat::zeros(8, 1, CV_32F);

			cv::Mat R, T, E, F;

			double errors;
			int flags = 0;
			
			if (two_step) {
				CLOG(LINFO) << "Performing two-step calibration";
				std::vector<cv::Mat> rvecs, tvecs;
				errors = cv::calibrateCamera(objectPoints_left, imagePoints_left, imageSize, cameraMatrix_left, distCoeffs_left, rvecs, tvecs);
				errors = cv::calibrateCamera(objectPoints_right, imagePoints_right, imageSize, cameraMatrix_right, distCoeffs_right, rvecs, tvecs);
				flags = cv::CALIB_FIX_INTRINSIC;
			} 
			
			errors = stereoCalibrate(objectPoints_left, imagePoints_left, imagePoints_right, 
				 cameraMatrix_left, distCoeffs_left, cameraMatrix_right, distCoeffs_right,
				 imageSize, R, T, E, F,
				 flags,
				 cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 1e-6)
			);
			
			// Display the results.
			LOG(LNOTICE) << "Calibration ended with reprojection error =" << errors;
			LOG(LNOTICE) << "Camera matrix (L): " << cameraMatrix_left;
			LOG(LNOTICE) << "Camera matrix (R): " << cameraMatrix_right;
			LOG(LNOTICE) << "Distortion coefficients (L): " << distCoeffs_left;
			LOG(LNOTICE) << "Distortion coefficients (R): " << distCoeffs_right;
			LOG(LNOTICE) << "Image size: " << imageSize;
			
			Types::CameraInfo camera_info_left;
			camera_info_left.setSize(imageSize);
			camera_info_left.setCameraMatrix(cameraMatrix_left);
			camera_info_left.setDistCoeffs(distCoeffs_left);
			camera_info_left.setRotationMatrix(cv::Mat::eye(3, 3, CV_64F));
			camera_info_left.setTranlationMatrix(cv::Mat::zeros(3, 1, CV_64F));
			
			Types::CameraInfo camera_info_right;
			camera_info_right.setSize(imageSize);
			camera_info_right.setCameraMatrix(cameraMatrix_right);
			camera_info_right.setDistCoeffs(distCoeffs_right);
			camera_info_right.setRotationMatrix(R);
			camera_info_right.setTranlationMatrix(T);

			// Write parameters to the camerainfo
			out_camerainfo_left.write(camera_info_left);
			out_camerainfo_right.write(camera_info_right);
    }
    else
			LOG(LERROR) << "Calib: dataset empty\n";

}

} //: namespace Calib
} //: namespace Processors
