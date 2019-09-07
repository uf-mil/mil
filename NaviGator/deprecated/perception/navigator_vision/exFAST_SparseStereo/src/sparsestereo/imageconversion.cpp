/*
 * Author: Konstantin Schauwecker
 * Year:   2012
 */

#include "imageconversion.h"
#include "simd.h"

namespace sparsestereo {
	using namespace cv;
	
	double ImageConversion::get8UScalingFactor(int type) {
		if(type == CV_16U)
			return 1.0/256.0;
		else if(type == CV_32F || type == CV_64F)
			return 255.0;
		else return 1.0;
	}
	
	double ImageConversion::get16UScalingFactor(int type) {
		if(type == CV_8U)
			return 256.0;
		else if(type == CV_32F || type == CV_64F)
			return 65535.0;
		else return 1.0;
	}
	
	bool ImageConversion::convert(const Mat& input, Mat* output, int channels, int type,
		double scalingFactor, int colorConv) {
		if(input.channels() == channels && input.type() == type) {
			// No conversion neccessary
			(*output) = input;
			return false;
		}
		else if(input.type() != type) {
			// Convert the data type
			Mat buffer;	
			input.convertTo(buffer, type, scalingFactor);
			if(input.channels() != channels)
				// Then convert color
				cvtColor(buffer, *output, colorConv);
			else (*output) = buffer;
		} else {
			// Only convert color
			cvtColor(input, *output, colorConv);
		}
		
		return true;
	}
	
	bool ImageConversion::convertTo8U(const Mat& input, cv::Mat_<unsigned char>* output) {
		return convert(input, output, 1, CV_8U, get8UScalingFactor(input.type()), CV_RGB2GRAY);
	}
	
	bool ImageConversion::convertTo16U(const Mat& input, cv::Mat_<unsigned short>* output) {
		return convert(input, output, 1, CV_16U, get8UScalingFactor(input.type()), CV_RGB2GRAY);
	}
	
	bool ImageConversion::convertToColor(const Mat& input, cv::Mat_<Vec3b>* output) {
		return convert(input, output, 3, CV_8U, get8UScalingFactor(input.type()), CV_GRAY2RGB);
	}
	
	void ImageConversion::unsignedToSigned(const Mat_<unsigned char>& src, Mat_<char>* dst) {
		// Adds an offset of -128 to all values
		if(src.cols % 16 == 0) {
			// SSE Optimized
			v16qi offset = SIMD::scalar16(-128);

			for(int y=0; y<src.rows; y++)
				for(int x=0; x<src.cols; x+=16)
					_mm_store_si128((__m128i*)&(*dst)(y,x), (__m128i)((v16qi)_mm_load_si128((__m128i*)&src(y, x)) + offset));
		}
		else {
			for(int y=0; y<src.rows; y++)
				for(int x=0; x<src.cols; x++)
					(*dst)(y,x) = src(y,x) - 128;	
		}
	}
	
	void ImageConversion::signedToUnsigned(const Mat_<char>& src, Mat_<unsigned char>* dst) {
		// Adds an offset of 128 to all values
		if(src.cols%16 != 0) {
			// SSE Optimized
			v16qi offset = SIMD::scalar16(128);

			for(int y=0; y<src.rows; y++)
				for(int x=0; x<src.cols; x+=16)
					_mm_store_si128((__m128i*)&(*dst)(y,x), (__m128i)((v16qi)_mm_load_si128((__m128i*)&src(y, x)) + offset));
		} else {
			for(int y=0; y<src.rows; y++)
				for(int x=0; x<src.cols; x++)
					(*dst)(y,x) = src(y,x) + 128;
		}
	}
}
