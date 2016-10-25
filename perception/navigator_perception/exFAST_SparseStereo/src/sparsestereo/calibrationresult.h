/*
 * Author: Konstantin Schauwecker
 * Year:   2012
 */

#ifndef SPARSESTEREO_CALIBRATIONRESULT_H
#define SPARSESTEREO_CALIBRATIONRESULT_H

#include <opencv2/opencv.hpp>

namespace sparsestereo {
	// Stores the results of a stereo camera calibration
	struct CalibrationResult {
		cv::Mat_<double> cameraMatrix[2], distCoeffs[2];
		cv::Mat_<double> R[2], P[2], Q, T;
		cv::Size imageSize;
		
		CalibrationResult() {
			cameraMatrix[0] = cv::Mat_<double>::eye(3, 3);
			cameraMatrix[1] = cv::Mat_<double>::eye(3, 3);
		}
		CalibrationResult(const char* file);
		
		void writeToFile(const char* file) const;
	};
}

#endif
