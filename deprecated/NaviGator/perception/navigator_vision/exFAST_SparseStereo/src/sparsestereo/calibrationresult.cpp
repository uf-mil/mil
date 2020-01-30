/*
 * Author: Konstantin Schauwecker
 * Year:   2012
 */

#include "calibrationresult.h"
#include <iostream>
#include "exception.h"

namespace sparsestereo {
	using namespace cv;
	using namespace std;
	
	CalibrationResult::CalibrationResult(const char* file) {
		FileStorage fs(file, CV_STORAGE_READ);
		if(!fs.isOpened())
			throw Exception("Unable to read calibration results");
		
		fs["M1"] >> cameraMatrix[0];
		fs["D1"] >> distCoeffs[0];
		fs["M2"] >> cameraMatrix[1];
		fs["D2"] >> distCoeffs[1];
		fs["R1"] >> R[0];
		fs["R2"] >> R[1];
		fs["P1"] >> P[0];
		fs["P2"] >> P[1];
		fs["Q"] >> Q;
		fs["T"] >> T;
		
		Mat_<int> sz(2, 1);
		fs["size"] >> sz;
		imageSize.width = sz(0, 0);
		imageSize.height = sz(1, 0);
		
		fs.release();
	}
	
	void CalibrationResult::writeToFile(const char * file) const {
		FileStorage fs(file, CV_STORAGE_WRITE);
		if(!fs.isOpened())
			throw Exception("Unable to store calibration results");
		
		fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0]
			<< "M2" << cameraMatrix[1] << "D2" << distCoeffs[1]
			<< "R1" << R[0] << "R2" << R[1] << "P1" << P[0] << "P2" << P[1] << "Q" << Q
			<< "T" << T;
			
		Mat_<int> sz(2, 1);
		sz(0, 0) = imageSize.width;
		sz(1, 0) = imageSize.height;
		fs << "size" << sz;
		
		fs.release();
	}	
}
