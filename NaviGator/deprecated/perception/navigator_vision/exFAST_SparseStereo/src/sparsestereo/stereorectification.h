/*
 * Author: Konstantin Schauwecker
 * Year:   2012
 */

#ifndef SPARSESTEREO_STEREORECTIFICATION_H
#define SPARSESTEREO_STEREORECTIFICATION_H

#include <utility>
#include <vector>
#include <cassert>
#include <opencv2/opencv.hpp>
#include <boost/smart_ptr.hpp>
#include "calibrationresult.h"

namespace sparsestereo {
	// Stores one epiline and provides element access
	class Epiline{
	public:
		Epiline(): line(NULL), dy(0) {}
	
		// Stores one real epiline	
		Epiline(const cv::Mat_<float>& refLines, const cv::Mat_<int> refLineIndices, const cv::Point2f& pointOnLine) {
			int index = refLineIndices(int(pointOnLine.y + 0.5), int(pointOnLine.x + 0.5));
			valid = (index != -1);
			if(valid) {
				assert(index != -1);
				dy = pointOnLine.y - refLines(index, (int)round(pointOnLine.x));
				line = &refLines(index, 0);
#ifndef NDEBUG
				width = refLines.cols;
#endif
			}
#ifndef NDEBUG
			else width = 0;
#endif
		}
		
		// Creates a dummy epiline
		Epiline(float y): line(dummyLine.get()), dy(y), valid(true) {
#ifndef NDEBUG
			width=dummyLineLength;
#endif
		}
		
		// Returns the y-coordinate for the epiline at position x
		float at(int x) const {
			assert(x >= 0 && x<width);
			return line[x] + dy;
		}
		
		// This method has to be called before the first dummy
		// epiline is created
		static void setMaxEpilineLength(int len);
		
		// Returns true if the current epiline is valid
		bool isValid() const {return valid;}
		
	private:
		static boost::scoped_array<float> dummyLine;
		static int dummyLineLength;
		const float* line;
		float dy;
#ifndef NDEBUG
		int width;
#endif
		bool valid;
	};

	// Class for rectifying input images
	class StereoRectification {
	public:
		enum Interpolation {
			Cubic = cv::INTER_CUBIC,
			Linear = cv::INTER_LINEAR,
			Nearest = cv::INTER_NEAREST
		};
	
		StereoRectification(const CalibrationResult& calibRes, Interpolation iterpolation = Linear);
		
		// Rectifies a stereo pair
		template <typename T>
		void rectifyStereoPair(const std::pair<cv::Mat_<T>, cv::Mat_<T> >& input, std::pair<cv::Mat_<T>, cv::Mat_<T> >* output) const {
			rectifyLeftImage(input.first, &(output->first));
			rectifyRightImage(input.second, &(output->second));
		}
		
		// Rectifies the left image
		template <typename T>
		void rectifyLeftImage(const cv::Mat_<T>& input, cv::Mat_<T>* output) const {
			cv::remap(input, *output, imageRectMap[0][0], imageRectMap[0][1], interpolation);
		}
		
		// Rectifies the right image
		template <typename T>
		void rectifyRightImage(const cv::Mat_<T>& input, cv::Mat_<T>* output) const {
			cv::remap(input, *output, imageRectMap[1][0], imageRectMap[1][1], interpolation);
		}
		
		// Rectifies a left integer point
		cv::Point2f rectifyLeftPoint(cv::Point2i inLeft) const {
			return pointRectMap[0](inLeft.y, inLeft.x);
		}
		
		// Rectifies a right integer point
		cv::Point2f rectifyRightPoint(cv::Point2i inRight) const {
			return pointRectMap[1](inRight.y, inRight.x); 
		}
		
		// Rectifies a left float point
		cv::Point2f rectifyLeftPoint(cv::Point2f inLeft) const {
			return interpolatedLookup(pointRectMap[0], inLeft);
		}
		
		// Rectifies a right float point
		cv::Point2f rectifyRightPoint(cv::Point2f inRight) const {
			return interpolatedLookup(pointRectMap[1], inRight);
		}
		
		// Rectifies a left float point. This implementation is slow!
		cv::Point2f highPrecisionRectifyLeftPoint(cv::Point2f inLeft) const;
		
		// Rectifies a right float point. This implementation is slow!
		cv::Point2f highPrecisionRectifyRightPoint(cv::Point2f inRight) const;
		
		// Returns an epiline going through the given left point
		Epiline getLeftEpiline(cv::Point2f p) const {
			return Epiline(epilines[0], epilineIndex[0], p);
		}
		
		// Returns an epiline going through the given left point
		Epiline getRightEpiline(cv::Point2f p) const {
			return Epiline(epilines[1], epilineIndex[1], p);
		}
		
		// Returns the calibration results
		const CalibrationResult& getCalibrationResult() const {return calibRes;}
		
	private:
		cv::Mat imageRectMap[2][2];
		cv::Mat_<cv::Point2f>  pointRectMap[2];
		cv::Mat_<float> epilines[2];
		cv::Mat_<int> epilineIndex[2];
		Interpolation interpolation;
		CalibrationResult calibRes;
		
		void initImageRectMap();
		void initPointRectMap();
		void initEpilines();
		
		// Preforms an bilinear interpolated lookup
		cv::Point2f interpolatedLookup(const cv::Mat_<cv::Point2f>& map, cv::Point2f pt) const {
			float dx = pt.x - int(pt.x);
			float dy = pt.y - int(pt.y);
			
			cv::Point2f top = (1.0 - dx) * map(int(pt.y), int(pt.x)) + dx * map(int(pt.y), int(pt.x + 1));
			cv::Point2f bottom = (1.0 - dx) * map(int(pt.y + 1), int(pt.x)) + dx * map(int(pt.y + 1), int(pt.x + 1));
			return (1.0 - dy) * top + dy * bottom;
		}
	};
}

#endif
