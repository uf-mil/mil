/*
 * Author: Konstantin Schauwecker
 * Year:   2012
 */

#ifndef SPARSESTEREO_CENSUS_H
#define SPARSESTEREO_CENSUS_H

#include <opencv2/opencv.hpp>
#include "simd.h"

namespace sparsestereo {
	// Computes various variants of the census transform
	class Census {
	public:
		// Census transform using 5x5 window
		template <typename T>
		static void transform5x5(const cv::Mat_<T>& input, cv::Mat_<unsigned int>* output);
		
	private:
		// SSE optimized implementations
		static void transform5x5SSE(const cv::Mat_<char>& input, cv::Mat_<unsigned int>* output);
		
		// Efficiently stores 4 byte blocks by interleaving four 1-byte vectors
		static __always_inline void storeSSEVec(const v16qi& byte4, const v16qi& byte3, const v16qi& byte2,
			const v16qi& byte1, char* dst);
	};
}

#endif
