/*
 * Author: Konstantin Schauwecker
 * Year:   2012
 */

#include <iostream>
#include <iomanip>
#include "extendedfast.h"
#include "fast9-inl.h"
#include "exception.h"

namespace sparsestereo {
	using namespace cv;
	using namespace std;
	
	const v16qi ExtendedFAST::const0(SIMD::scalar16NonLookup(0));
	const v16qi ExtendedFAST::const128(SIMD::scalar16NonLookup(128));
	const v16qi ExtendedFAST::const255(SIMD::scalar16NonLookup(255));
	
	// Longest arc length lookup table
	unsigned char ExtendedFAST::lookupTable[1U<<16];
	bool ExtendedFAST::lookupInitialized = false;
	
	ExtendedFAST::ExtendedFAST(bool nonmaxSuppression, unsigned char minThreshold, float adaptivity, bool subpixelPrecision, int minBorder)
		: nonmaxSuppression(nonmaxSuppression), minThreshold(minThreshold), adaptivity(adaptivity), subpixelPrecision(subpixelPrecision) {
		const int reserve = 512;
		cornersMin.reserve(reserve); cornersAdapt.reserve(reserve); scores.reserve(reserve);
		
		if(!lookupInitialized) {
			// The first instance initializes the lookup table.
			// Should be threadsafe as we always write the same values and set the
			// initialization flag last.
			for(int i=0; i<=0xFFFF; i++)
				lookupTable[i] = findLongestArc(i) >= 9 ? 0xFF : 0;
			lookupInitialized = true;
		}
	}
	
	ExtendedFAST::~ExtendedFAST() {
	}
	
	inline unsigned char ExtendedFAST::findLongestArc(unsigned short stripe) {
		int bestLength = 0;
		int startLength = -1;
		int currentLength = 0;
		
		// We iterate over all possible 16-bit permutations
		for(int i=1; i<0xFFFF; i = i<<1) {
			if(stripe & i)
				// Inside an arc segment
				currentLength++;
			else {
				// Outside an arc segment
				if(currentLength > bestLength)
					bestLength = currentLength;
				if(startLength == -1)
					startLength = currentLength;
				currentLength = 0;
			}
		}
		
		// wrap-around case
		if(startLength != -1)
			currentLength += startLength;
		
		// Handle last arc segment
		if(currentLength > bestLength)
			bestLength = currentLength;
		
		return bestLength;
	}
	
	void ExtendedFAST::initOffsets(int step) {
		offsets[0] = step*-3 -1;
		offsets[1] = step*-3;
		offsets[2] = step*-3 +1;
		offsets[3] = step*-2 +2;
		offsets[4] = step*-1 +3;
		offsets[5] = +3;
		offsets[6] = step +3;
		offsets[7] = step*+2 +2;
		offsets[8] = step*+3 +1;
		offsets[9] = step*+3;
		offsets[10] = step*+3 -1;
		offsets[11] = step*+2 -2;
		offsets[12] = step*+1 -3; //Aligned
		offsets[13] = -3; //Aligned
		offsets[14] = step*-1 -3; //Aligned
		offsets[15] = step*-2 -2;
	}

	void ExtendedFAST::detectImpl(const Mat& image, vector<KeyPoint>& keypoints, const Mat& mask) {
		
		if(mask.data != NULL)
			throw Exception("Feature detection masks not supported!");
		else if(image.type() != CV_8U)
			throw Exception("Image data has to be of type unsigned char!");
		
		// Create offsets for circle pixel
		initOffsets(image.step);
		
		FAST(image, cornersMin, minThreshold, false);
		adaptiveThresholdAndScore(image);	
	 
		if(nonmaxSuppression)
		{
			// Perform nonmax suppression
			vector<int> nonmaxPoints;
			fast9.nonMaxSuppression(cornersAdapt, scores, nonmaxPoints);
			
			// Copy and optionally refine result
			keypoints.reserve(nonmaxPoints.size());
			for(unsigned int i=0; i<nonmaxPoints.size(); i++) {
				int index = nonmaxPoints[i];
				Point2f pt = Point2f(cornersAdapt[index].x, cornersAdapt[index].y);
				keypoints.push_back(KeyPoint(pt, 6.f, -1.f, scores[index]));
			}
		}
		else
		{   
			// Copy everything
			size_t i, n = cornersAdapt.size();
			keypoints.resize(n);
			for( i = 0; i < n; i++ )
				keypoints[i] = KeyPoint(cornersAdapt[i], 6.f, -1.f, -1000);
		}
		
		// Clear buffers
		cornersMin.clear(); cornersAdapt.clear(); scores.clear();
	}
	
	void ExtendedFAST::adaptiveThresholdAndScore(const Mat_<unsigned char>& input) {
		initOffsets((int)input.step);
		for(unsigned int i=0; i<cornersMin.size(); i++) {
			Point2i pt = cornersMin[i].pt;
			if(testFeatureAndScore(input, pt, nonmaxSuppression))
				cornersAdapt.push_back(pt);
		}
	}
	
	__always_inline v16qi ExtendedFAST::loadCircleSSE(const cv::Mat_<unsigned char>& input, int x, int y) {
		const char* center = (const char*)&(input(y, x));
		
		v16qi circle = {*(center + offsets[0]), *(center + offsets[1]), *(center + offsets[2]), *(center + offsets[3]), 
			*(center + offsets[4]), *(center + offsets[5]), *(center + offsets[6]), *(center + offsets[7]), 
			*(center + offsets[8]), *(center + offsets[9]), *(center + offsets[10]), *(center + offsets[11]), 
			*(center + offsets[12]), *(center + offsets[13]), *(center + offsets[14]), *(center + offsets[15]) 
		};
		
		return circle;
	}

	__always_inline bool ExtendedFAST::testFeatureAndScore(const cv::Mat_<unsigned char>& input, const cv::Point2i pt, bool storeScore) {
		// Calculate the adaptive threshold by computing an RMS contrast
		// like measure based on absolute values
		v16qi uCircle = loadCircleSSE(input, (int)pt.x, (int)pt.y);
		v16qi circle = uCircle - const128;
		v2di partialSum = __builtin_ia32_psadbw128(uCircle, const0);
		int sum = SIMD::element2(partialSum, 0) + SIMD::element2(partialSum, 1);
		v16qi avg = SIMD::scalar16(sum/16);
		
		v2di partialSAD = __builtin_ia32_psadbw128(uCircle, avg);
		int sad = SIMD::element2(partialSAD, 0) + SIMD::element2(partialSAD, 1);
		unsigned char adaptiveThreshold = cv::saturate_cast<char>(sad * adaptivity / 16);
		v16qi adaptiveThresholdVec = SIMD::scalar16(adaptiveThreshold);
		
		// Perform corner test
		v16qi center = SIMD::scalar16(SPARSESTEREO_EXFAST_CENTRAL_VALUE(input, (int)pt.y, (int)pt.x) - 128);
		if(detectSingleCornerSSE(circle, center, adaptiveThresholdVec, 9)) {
			if(storeScore) // We have to calculate a corner score
				scores.push_back(calcSingleScoreSSE(circle, center, adaptiveThresholdVec, 9) - adaptiveThreshold);
			return true;
		}
		else return false;
	}
	
	__always_inline bool ExtendedFAST::detectSingleCornerSSE(const v16qi& circle, const v16qi& center, const v16qi& threshold,
		unsigned char minLength) {
		
		// Find longest brighter arc
		v16qi centersPlus = __builtin_ia32_paddsb128(center, threshold);
		int arcBrighter = __builtin_ia32_pmovmskb128(__builtin_ia32_pcmpgtb128(circle, centersPlus));
				
		if(lookupTable[arcBrighter] >= minLength)
			return true;
		else {
			// Find longest darker arc
			v16qi centersMinus = __builtin_ia32_psubsb128(center, threshold);
			int arcDarker = __builtin_ia32_pmovmskb128(__builtin_ia32_pcmpgtb128(centersMinus, circle));
			if(lookupTable[arcDarker] >= minLength)
				return true;
		}
		
		return false;
	}
	
	__always_inline unsigned char ExtendedFAST::calcSingleScoreSSE(const v16qi& circle, v16qi center, const v16qi& bstartVec,
		unsigned char minLength) {
		
		v16qi bmin = bstartVec;
		v16qi bmax = const255;
		v16qi b = __builtin_ia32_pavgb128 (bmax, bmin);
		center += const128; 
		
		//Compute the score using binary search
		for(;;)
		{
			// Find brighter arc
			v16qi centerPlus = __builtin_ia32_paddusb128(center, b) - const128;
			int arcBrighter = __builtin_ia32_pmovmskb128(__builtin_ia32_pcmpgtb128(circle, centerPlus));
			
			if(lookupTable[arcBrighter] >= minLength)
				bmin = b; // corner
			else {
				// Find darker arc
				v16qi centerMinus = __builtin_ia32_psubusb128(center, b) - const128;
				int arcDarker = __builtin_ia32_pmovmskb128(__builtin_ia32_pcmpgtb128(centerMinus, circle));
				if(lookupTable[arcDarker] >= minLength)
					bmin = b; // corner
				else
					bmax = b; // Not a corner
			}
			
			unsigned char singleBMin = SIMD::element16(bmin, 0), singleBMax = SIMD::element16(bmax, 0);
			if(singleBMin == singleBMax || singleBMin == singleBMax - 1)
				return (unsigned char)SIMD::element16(bmin, 0);
			
			// Update threshold	
			b = __builtin_ia32_pavgb128 (bmax, bmin);
		}
	}
}
