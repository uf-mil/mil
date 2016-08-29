#ifndef SHAPE_DETECTOR_H
#define SHAPE_DETECTOR_H
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
class ShapeDetector {
	private:
	static float findAngle(cv::Point p1, cv::Point p2, cv::Point p3);
	static float chisquared(std::vector<float> observed, float expected);
	static float findMean(std::vector<float> observed);
	static float findVarience(std::vector<float> observed);
	public:
	//Not the best way... but meh, customization
	static bool boundingAreaCross(std::vector<cv::Point> &points);
	static bool boundingAreaTriangle(std::vector<cv::Point> &points);
	static bool boundingAreaCircle(std::vector<cv::Point> &points); //Either with points or with Mat?
	
	static bool angleTestCross(std::vector<cv::Point> &points);
	static bool angleTestTriangle(std::vector<cv::Point> &points);
	static bool angleTestCirlce(std::vector<cv::Point> &points); //Maybe
	
	static bool testRatioAreaPerimeterCircle(std::vector<cv::Point> &points);
//	static bool testRatioAreaPerimeterTriangle(std::vector<cv::Point> &points);
	static bool testRatioAreaPerimeterCross(std::vector<cv::Point> &points);
	static bool testPointAlignmentTriangle(std::vector<cv::Point> &points);
	
	static bool isCross(std::vector<cv::Point> &points);
	static bool isTriangle(std::vector<cv::Point> &points);
	static bool isCircle(std::vector<cv::Point> &points);
	
};

#endif
