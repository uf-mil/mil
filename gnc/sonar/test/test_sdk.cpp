/*
 * File Sonar Example using OpenCV
 * Demonstrate opening a file, accessing a head, and retriving a ping.
 * The ping is then processed into an image and displayed using OpenCV
 * Finally, a colormap is loaded and the image is colormapped.  The
 * color image is also displayed with OpenCV
 */

#include <stdio.h>
#include <iostream>
#include <string>


#include "BVWrapper.h"

#include <opencv2/core/core.hpp> 

int main( void )
{	
	const char bad[] = "/home/tess/bvtsdk/data/swimmer.son";
	try{
		auto sonar = bv::BVWrapper(bv::FILE, bad);
		cv::Mat imagebw8;
		cv::Mat imagebw16;
		cv::Mat imagecolor;
		while(sonar.getNextPing(imagebw8, imagebw16, imagecolor)){
			cv::imshow("image", imagecolor);
			cv::waitKey(0);
		}
	}catch(char const* msg){
		std::cout<< msg << std::endl;
	}
	
}