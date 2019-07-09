/*
 * Author: Konstantin Schauwecker
 * Year:   2012
 */

#include "extendedfast.h"

namespace sparsestereo {

	template <typename T>
	__always_inline v16qi ExtendedFAST::loadCircleSSE(const cv::Mat_<unsigned char>& input, int x, int y) {
		const char* center = (const char*)&(input(y, x));
		
		v16qi circle = {*(center + offsets[0]), *(center + offsets[1]), *(center + offsets[2]), *(center + offsets[3]), 
			*(center + offsets[4]), *(center + offsets[5]), *(center + offsets[6]), *(center + offsets[7]), 
			*(center + offsets[8]), *(center + offsets[9]), *(center + offsets[10]), *(center + offsets[11]), 
			*(center + offsets[12]), *(center + offsets[13]), *(center + offsets[14]), *(center + offsets[15]) 
		};
		
		return circle;
	}

	// Tests for a feature and optionally stores the score
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
}
