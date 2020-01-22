/*
 * Author: Konstantin Schauwecker
 * Year:   2012
 */

#ifndef SPARSESTEREO_SPARSEMATCH_H
#define SPARSESTEREO_SPARSEMATCH_H

#include <opencv2/opencv.hpp>
#include "stereorectification.h"
#include "exception.h"

namespace sparsestereo {
	// Struct for a single sparse stereo match
	struct SparseMatch {
		SparseMatch()
			: imgLeft(NULL), imgRight(NULL), rectLeft(), rectRight(), cost(0) {}
		SparseMatch(const cv::KeyPoint* imgLeft, const cv::KeyPoint* imgRight,
			const cv::Point2f& rectLeft, const cv::Point2f& rectRight, short cost = 0)
			: imgLeft(imgLeft), imgRight(imgRight), rectLeft(rectLeft), rectRight(rectRight), cost(cost) {}
		SparseMatch(const cv::KeyPoint* imgLeft, const cv::KeyPoint* imgRight,  short cost = 0)
			: imgLeft(imgLeft), imgRight(imgRight), rectLeft(imgLeft->pt), rectRight(imgRight->pt), cost(cost) {}
		
		const cv::KeyPoint* imgLeft;
		const cv::KeyPoint* imgRight;
		cv::Point2f rectLeft;
		cv::Point2f rectRight;
		short cost;
		
		float disparity() const {
			return rectLeft.x - rectRight.x;
		}
		
		// Converts a set of image coordinates and disparity values to 3d points
		static void projectMatches(const std::vector<SparseMatch>& matches, std::vector<cv::Point3f>* points,
			const StereoRectification* rect) {
			if(rect != NULL) {
				const CalibrationResult& calib = rect->getCalibrationResult();
				cv::Mat_<float> qFloat(calib.Q.rows, calib.Q.cols);
				calib.Q.convertTo(qFloat, CV_32F);
			
				points->reserve(matches.size());
				for(unsigned int i=0; i<matches.size(); i++) {
					float point[4] = {matches[i].rectLeft.x, matches[i].rectLeft.y, matches[i].disparity(), 1.0};
					cv::Mat_<float> product = qFloat * cv::Mat_<float>(4, 1, point);
					points->push_back(cv::Point3f(product(0)/product(3), product(1)/product(3), product(2)/product(3)));
				}
			}
			else throw Exception("Projection of 3d points is only possible for calibrated cameras.");
		}
	};
}

#endif
