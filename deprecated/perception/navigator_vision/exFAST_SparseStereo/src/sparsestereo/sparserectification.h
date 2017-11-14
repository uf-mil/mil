/*
 * Author: Konstantin Schauwecker
 * Year:   2012
 */

#ifndef SPARSESTEREO_SPARSERECTIFICATION_H
#define SPARSESTEREO_SPARSERECTIFICATION_H

#include <vector>
#include <opencv2/opencv.hpp>

namespace sparsestereo {
	class StereoRectification;

	// Rectifies two sets of features from the left and right image
	class SparseRectification {
	public:
		// Stores a pair of rectified and unrectified feature points
		struct RectifiedFeature {
			const cv::KeyPoint* imgPoint;
			cv::Point2f rectPoint;
		};
	
		SparseRectification(bool subpixelFeatures, StereoRectification* rect);
		
		// Rectifies a set of sparse features
		void rectify(const std::vector<cv::KeyPoint>& inLeft, const std::vector<cv::KeyPoint>& inRight,
			std::vector<RectifiedFeature>* outLeft, std::vector<RectifiedFeature>* outRight);
			
		// Precomputes epilines start position
		void precomputeEpilinesStart(int imageWidth, int imageHeight, cv::Mat_<short int>* dst);
	
	private:
		// Comparision operator used for sorting feature points
		static bool featureSortComp(const RectifiedFeature& left, const RectifiedFeature& right) {
			return left.rectPoint.y < right.rectPoint.y || (left.rectPoint.y == right.rectPoint.y &&
					left.rectPoint.x < right.rectPoint.x);
		}
		
		bool subpixelFeatures;
		StereoRectification* rect;
		
		// Estimates the distorted point in the left image, for which the disparity between rectified left and
		// right image is 0. The given left point just serves for a first estimate.	
		inline float estimateDistortedInfiniteLeftX(int leftImgY, int rightRectX);
	};
}

#endif
