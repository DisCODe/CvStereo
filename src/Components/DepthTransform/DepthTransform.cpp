/*!
 * \file
 * \brief
 * \author Łukasz Żmuda
 */

#include <memory>
#include <string>

#include "DepthTransform.hpp"
#include "Common/Logger.hpp"
#include <Types/MatrixTranslator.hpp>
#include "Property.hpp"

#include <boost/bind.hpp>
#include <boost/format.hpp>

namespace Processors {
namespace DepthTransform {
  
using Types::HomogMatrix;

DepthTransform::DepthTransform(const std::string & name) :
		Base::Component(name)
{



}

DepthTransform::~DepthTransform() {

}

void DepthTransform::prepareInterface() {

	// Register data streams, events and event handlers HERE!
	registerStream("in_homogMatrix", &in_homogMatrix);
	registerStream("in_image_xyz", &in_image_xyz);
	registerStream("out_image_xyz", &out_image_xyz);

	// Register handlers
	h_DepthTransformation.setup(boost::bind(&DepthTransform::DepthTransformation, this));
	registerHandler("DepthTransformation", &h_DepthTransformation);
	addDependency("DepthTransformation", &in_image_xyz);

}

bool DepthTransform::onInit() {

	return true;
}

bool DepthTransform::onFinish() {
	return true;
}

bool DepthTransform::onStop() {
	return true;
}

bool DepthTransform::onStart() {
	return true;
}

void DepthTransform::DepthTransformation() {
	try{
	  
	  
	
	  
	  cv::Mat depth_image = in_image_xyz.read();
	  HomogMatrix hm = in_homogMatrix.read();
	  
	  stringstream ss;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 4; ++j) {
			ss << hm.elements[i][j] << "  ";
		}
	}
	CLOG(LINFO) << "HomogMatrix:\n" << ss.str() << endl;
	  
	  
	  
	  cv::Size depth_size = depth_image.size();

	  cv::Mat_<double> rotationMatrix(3,3);
	  cv::Mat_<double> tvec(3,1);	  
	  
	  cv::Mat tmp_img;// = cv::Mat(depth_image);
	  tmp_img.create(depth_size, CV_32FC3);
	    
            float newX, newY, newZ;
            depth_size.width *= 3;
	    
	    
	    rotationMatrix.at<double>(0,0)=hm.elements[0][0];
	    rotationMatrix.at<double>(0,1)=hm.elements[0][1];
	    rotationMatrix.at<double>(0,2)=hm.elements[0][2];
	    
	    rotationMatrix.at<double>(1,0) = hm.elements[1][0]; 
	    rotationMatrix.at<double>(1,1) = hm.elements[1][1];
	    rotationMatrix.at<double>(1,2) = hm.elements[1][2];
	    
	    rotationMatrix.at<double>(2,0)=hm.elements[2][0];
	    rotationMatrix.at<double>(2,1)=hm.elements[2][1];
	    rotationMatrix.at<double>(2,2)=hm.elements[2][2];
	   
	    tvec.at<double>(0,0) = hm.elements[0][3];
	    tvec.at<double>(1,0) = hm.elements[1][3];
	    tvec.at<double>(2,0) = hm.elements[2][3];
	    
	    LOG(LINFO)<< " rotationMatrix " << rotationMatrix;
	    
	    LOG(LINFO)<<"hm" <<hm.elements[3][0] << hm.elements[3][1]<<hm.elements[3][2];

	   
	    LOG(LINFO)<< " tvec " << tvec;
	   
	    rotationMatrix=rotationMatrix.t();
	    

	//LOG(LINFO) << "DepthTransformation\n";
        for (int i = 0; i < depth_size.height; i++)
        {
	  
            const float* depth_ptr = depth_image.ptr <float> (i);
	    float* depth_ptr_tmp = tmp_img.ptr <float> (i);
            int j, k = 0;
	    int val = 0;
	   
            for (j = 0; j < depth_size.width; j += 3)
            {
	      
		 // get x, y, z from depth_image
		 float x = depth_ptr[j];
                 float y = depth_ptr[j + 1];
		 float z= depth_ptr[j + 2];
		 
		  if(z==10000)
		  {
		      depth_ptr_tmp[j]=x;
		      depth_ptr_tmp[j + 1]=y;
		      depth_ptr_tmp[j + 2]=z;
		      continue;
		  }
		
		x=x-tvec.at<double>(0,0);
		y=y-tvec.at<double>(1,0);
		z=z-tvec.at<double>(2,0);

      		newX = x*rotationMatrix.at<double>(0,0) + y*rotationMatrix.at<double>(0,1)+z*rotationMatrix.at<double>(0,2);
		newY = x*rotationMatrix.at<double>(1,0) + y*rotationMatrix.at<double>(1,1)+z*rotationMatrix.at<double>(1,2);
		newZ = x*rotationMatrix.at<double>(2,0) + y*rotationMatrix.at<double>(2,1)+z*rotationMatrix.at<double>(2,2);

		depth_ptr_tmp[j]=newX;
                depth_ptr_tmp[j + 1]=newY;
                depth_ptr_tmp[j + 2]=newZ;
		
            }
        }
       
	  LOG(LINFO) << "Writing to data stream";
	  out_image_xyz.write(tmp_img);
	  
	} catch (...)
	{
		LOG(LERROR) << "DepthTransformation:Error occured in processing input";
	}
}


} //: namespace DepthTransform
} //: namespace Processors