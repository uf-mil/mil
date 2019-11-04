/*
 * Author: Konstantin Schauwecker
 * Year:   2012
 */

#ifndef SPARSESTEREO_SPARSESTEREO_H
#define SPARSESTEREO_SPARSESTEREO_H

#include <vector>
#include <utility>
#include <opencv2/opencv.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/tuple/tuple.hpp>
#include "sparsematch.h"
#include "sparserectification.h"

namespace sparsestereo {
	class StereoRectification;
	
	// Class bundling sparse stereo algorithms	
	template <class CORRELATION, typename COST_TYPE>
	class SparseStereo {
	public:	
		SparseStereo(int maxDisparity, float yTolerance = 1, float uniqueness = 0.6, StereoRectification* rect = NULL,
			bool subpixelFeatures = false, bool storeUnmatched = false, int leftRightStep = 1);
		~SparseStereo();
		
		// Matches using a census window
		void match(const cv::Mat& left, const cv::Mat& right, const std::vector<cv::KeyPoint>& leftFeat,
			const std::vector<cv::KeyPoint>& rightFeat, std::vector<SparseMatch>* matches);
		
	private:
		std::vector<SparseRectification::RectifiedFeature> leftFeatures, rightFeatures;	
		int maxDisparity;
		float yTolerance;
		float uniqueness;
		StereoRectification* rect;
		bool storeUnmatched;
		int leftRightStep;
		std::vector<std::pair<int, COST_TYPE> > minimumMatches;
		cv::Mat_<short int> precompEpilinesStart; // Preocomputed epilines start positions
		cv::Size frameSize;
		SparseRectification sparseRect;
		
		// Gets the starting offsets of each row and returns the maximum row length
		int getRowOffsets(const std::vector<SparseRectification::RectifiedFeature>& features, unsigned int* offsets,
			int maxRows);
			
		// Calculates the matching costs using census windows
		void calcCosts(const cv::Mat& left, const cv::Mat& right, unsigned int* rowOffsets);
			
		// Performs a left/right consistency check that is dense in the left image
		void denseConsistencyCheck(const cv::Mat& left, const cv::Mat& right);
	};
}

#endif
