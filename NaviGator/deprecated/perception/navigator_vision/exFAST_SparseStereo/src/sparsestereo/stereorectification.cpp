/*
 * Author: Konstantin Schauwecker
 * Year:   2012
 */

#include "stereorectification.h"
#include <cstring>
#include <climits>

namespace sparsestereo {
	using namespace std;
	using namespace cv;
	using namespace boost;
	
	scoped_array<float> Epiline::dummyLine;
	int  Epiline::dummyLineLength = -1;
	
	void Epiline::setMaxEpilineLength(int len) {
		if(dummyLineLength != len) {
			dummyLine.reset(new float[len]);
			memset(dummyLine.get(), 0, sizeof(float)*len);
			dummyLineLength = len;
		}
	}
	
	StereoRectification::StereoRectification(const CalibrationResult& calibRes, Interpolation interpolation)
		: interpolation(interpolation), calibRes(calibRes)
	{
		initImageRectMap();
		initPointRectMap();
		initEpilines();
	}
	
	void StereoRectification::initImageRectMap() {
		// Create rectification maps for image rectification
		initUndistortRectifyMap(calibRes.cameraMatrix[0], calibRes.distCoeffs[0], calibRes.R[0],
			calibRes.P[0], calibRes.imageSize, CV_16SC2, imageRectMap[0][0], imageRectMap[0][1]);
		initUndistortRectifyMap(calibRes.cameraMatrix[1], calibRes.distCoeffs[1], calibRes.R[1],
			calibRes.P[1], calibRes.imageSize, CV_16SC2, imageRectMap[1][0], imageRectMap[1][1]);
	}
	
	void StereoRectification::initPointRectMap() {
		// Create rectification maps for integer point rectification
		// Collect all image point coordinates
		vector<Point2f> distortedPoints((calibRes.imageSize.width + 1) * (calibRes.imageSize.height + 1));
		for(int y=0; y<=calibRes.imageSize.height; y++)
			for(int x=0; x<=calibRes.imageSize.width; x++)
				distortedPoints[y * (calibRes.imageSize.width + 1) + x]/*, 0)*/ = Point2f(x, y);
		for(int i=0; i<2; i++) {
			// Perform rectification
			vector<Point2f> undistortedPoints(distortedPoints.size());			
			undistortPoints(distortedPoints, undistortedPoints, calibRes.cameraMatrix[i], calibRes.distCoeffs[i],
				calibRes.R[i], calibRes.P[i]);
			// Store results
			pointRectMap[i] = Mat_<Point2f>(calibRes.imageSize.height + 1, calibRes.imageSize.width + 1, Point2f(-1, -1));
			for(int y=0; y<= calibRes.imageSize.height; y++)
				for(int x=0; x<= calibRes.imageSize.width; x++)
					pointRectMap[i](y,x) = undistortedPoints[y * (calibRes.imageSize.width + 1) + x];//, 0);
		}
	}
	
	void StereoRectification::initEpilines() {
		for(int i=0; i<2; i++) {
			epilines[i] = Mat_<float>(calibRes.imageSize, -1e6);
			// First calculate a float undistortion map
			Mat_<float> rectMap[2];
			initUndistortRectifyMap(calibRes.cameraMatrix[i], calibRes.distCoeffs[i], calibRes.R[i],
				calibRes.P[i], calibRes.imageSize, CV_32F, rectMap[0], rectMap[1]);
		
			//Calculate epilines
			for(int y=0; y < calibRes.imageSize.height; y++) {
				for(int x=0; x<calibRes.imageSize.width - 1; x++) {
					// Linearly interpolate between each entry in the undistortion map
					double dx = rectMap[0](y, x + 1) - rectMap[0](y, x);
					double dy = rectMap[1](y, x + 1) - rectMap[1](y, x);
					
					for(int imgX = (int)round(rectMap[0](y, x)); imgX <= (int)round(rectMap[0](y, x+1)); imgX++) {
						if(imgX >=0 && imgX < calibRes.imageSize.width) {
							double interpolated = rectMap[1](y, x) + dy / dx * (imgX - rectMap[0](y, x));
							if(interpolated >= 0 && interpolated < calibRes.imageSize.height) {
								epilines[i](y, imgX) = interpolated;
							}
						}
					}
				}
			}
			
			// Fill epilines index
			epilineIndex[i] = Mat_<int>(calibRes.imageSize, -1);
			for(int line = 0; line < calibRes.imageSize.height - 1; line++)
				for(int x=0; x<calibRes.imageSize.width; x++) {
					for(int imgY = max(0, int(epilines[i](line, x)+0.5)); imgY <= min(calibRes.imageSize.height-1, (int)round(epilines[i](line+1, x))); imgY++)
						if(imgY >=0 && imgY < calibRes.imageSize.height) {
							if(fabs(imgY - epilines[i](line, x)) < fabs(imgY - epilines[i](line+1, x)))
								epilineIndex[i](imgY, x) = line;
							else epilineIndex[i](imgY, x) = line + 1;
						}
				}
		}		
	}
	
	Point2f StereoRectification::highPrecisionRectifyLeftPoint(Point2f inLeft) const {
		Mat_<Point2f> inLeftPoint(1, 1, inLeft);
		vector<Point2f> outLeftPoint;
		
		undistortPoints(inLeftPoint, outLeftPoint, calibRes.cameraMatrix[0], calibRes.distCoeffs[0],
			calibRes.R[0], calibRes.P[0]);
		
		return outLeftPoint[0];
	}
	
	Point2f StereoRectification::highPrecisionRectifyRightPoint(Point2f inRight) const {		
		Mat_<Point2f> inRightPoint(1, 1, inRight);
		vector<Point2f> outRightPoint;
		
		undistortPoints(inRightPoint, outRightPoint, calibRes.cameraMatrix[1], calibRes.distCoeffs[1],
			calibRes.R[1], calibRes.P[1]);
		
		return outRightPoint[0];
	}
}
