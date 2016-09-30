/*
 * Author: Konstantin Schauwecker
 * Year:   2012
 */

#ifndef SPARSESTEREO_IMAGECONVERSION_H
#define SPARSESTEREO_IMAGECONVERSION_H

#include <opencv2/opencv.hpp>

namespace sparsestereo {
	// Static methods for image conversion. If the return value is false
	// then the output image points to the data of the input image. In this
	// case, the input image should not be deleted.
	class ImageConversion {
	public:
		// Converts an image to 8-bit grayscale
		static bool convertTo8U(const cv::Mat& input, cv::Mat_<unsigned char>* output);
		// Converts an image to 16-bit grayscale
		static bool convertTo16U(const cv::Mat& input, cv::Mat_<unsigned short>* output);
		// Converts an image to a color image
		static bool convertToColor(const cv::Mat& input, cv::Mat_<cv::Vec3b>* output);
		// Converts an unsigned char to a signed char image
		static void unsignedToSigned(const cv::Mat_<unsigned char>& src,cv::Mat_<char>* dst);
		// Converts a signed char to an unsigned char image
		static void signedToUnsigned(const cv::Mat_<char>& src,cv::Mat_<unsigned char>* dst);
		
	private:
		// Methods for calculating scaling factors
		static double get8UScalingFactor(int type);
		static double get16UScalingFactor(int type);
		static bool convert(const cv::Mat& input, cv::Mat* output, int channels, int type,
			double scalingFactor, int colorConv);
	};
}

#endif
