/*
 * Author: Konstantin Schauwecker
 * Year:   2012
 */

#ifndef SPARSESTEREO_EXTENDEDFAST_H
#define SPARSESTEREO_EXTENDEDFAST_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include "simd.h"
#include "fast9.h"

namespace sparsestereo {

//#define SPARSESTEREO_EXFAST_CENTRAL_VALUE(img, y, x) img(y, x) 
#define SPARSESTEREO_EXFAST_CENTRAL_VALUE(img, y, x) (((int)img(y, x) + (int)img(y-1, x) + (int)img(y+1, x) + (int)img(y, x-1) + (int)img(y, x+1))/5)

	// Extended version of the fast feature detector
	class ExtendedFAST: public cv::FeatureDetector {
	public:
		ExtendedFAST(bool nonmaxSuppression, unsigned char minThreshold, float adaptivity, bool subpixelPrecision, int minBorder = 0);
		virtual ~ExtendedFAST();
			
	protected:
		// Implements the feature detector interface
		virtual void detectImpl(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask=cv::Mat()) const {
			// OpenCV tries to force this method to be const, but we don't like that!
			const_cast<ExtendedFAST*>(this)->detectImpl(image, keypoints, mask);
		}
		void detectImpl(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask=cv::Mat());
	
	private:
		// Constants for SSE processing
		static const v16qi const0, const128, const255;
	
		bool nonmaxSuppression;
		// Adaptive threshold parameteres
		unsigned char minThreshold;
		unsigned char maxThreshold;
		float adaptivity;
		bool subpixelPrecision;
		int border;
		FAST9<unsigned char> fast9;
		
		// Feature point data
		std::vector<cv::Point2i> cornersAdapt;
		std::vector<cv::KeyPoint> cornersMin;
		std::vector<unsigned char> scores;
		
		// Offsets for the circle pixel
		int offsets[16];
		// Lookup table for longest arc lengths
		static unsigned char lookupTable[1U<<16];
		static bool lookupInitialized;
		
		// Finds the longest arc for a given comparison pattern
		inline unsigned char findLongestArc(unsigned short stripe);
		// Initializes offsets for corner detection
		void initOffsets(int step);
		// Loads 16 circle pixels into one SSE vector
		__always_inline v16qi loadCircleSSE(const cv::Mat_<unsigned char>& input, int x, int y);
		// Performs feature test and score calculation
		__always_inline bool testFeatureAndScore(const cv::Mat_<unsigned char>& input, const cv::Point2i pt, bool storeScore);
		// Performs a corner detection using the circle pixels from an SSE vector
		__always_inline bool detectSingleCornerSSE(const v16qi& circle, const v16qi& center, const v16qi& threshold,
			unsigned char minLength);
		// Performs a score calculation using the circle pixels from an SSE vector
		__always_inline unsigned char calcSingleScoreSSE(const v16qi& circle, v16qi center, const v16qi& bstartVec,
			unsigned char minLength);
		// Applies the adaptive threshold to cornersMin, and calculates the score
		void adaptiveThresholdAndScore(const cv::Mat_<unsigned char>& input);
	};
}

#endif
