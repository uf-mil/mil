/*
 * Author: Konstantin Schauwecker
 * Year:   2012
 */
 
// This is a minimalistic example on how to use the extended
// FAST feature detector and the sparse stereo matcher.

#include <opencv2/opencv.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <vector>
#include <iostream>
#include <sparsestereo/exception.h>
#include <sparsestereo/extendedfast.h>
#include <sparsestereo/stereorectification.h>
#include <sparsestereo/sparsestereo-inl.h>
#include <sparsestereo/census-inl.h>
#include <sparsestereo/imageconversion.h>
#include <sparsestereo/censuswindow.h>

using namespace std;
using namespace cv;
using namespace sparsestereo;
using namespace boost;
using namespace boost::posix_time;

int main(int argc, char** argv) {
	try {
		// Stereo matching parameters
		double uniqueness = 0.7;
		int maxDisp = 70;
		int leftRightStep = 2;
		
		// Feature detection parameters
		double adaptivity = 1.0;
		int minThreshold = 10;
		
		// Parse arguments
		if(argc != 3 && argc != 4) {
			cout << "Usage: " << argv[0] << " LEFT-IMG RIGHT-IMG [CALIBRARION-FILE]" << endl;
			return 1;
		}
		char* leftFile = argv[1];
		char* rightFile = argv[2];
		char* calibFile = argc == 4 ? argv[3] : NULL;

		// Read input images
		cv::Mat_<unsigned char> leftImg, rightImg;
		leftImg = imread(leftFile, CV_LOAD_IMAGE_GRAYSCALE);
		rightImg = imread(rightFile, CV_LOAD_IMAGE_GRAYSCALE);
		
		if(leftImg.data == NULL || rightImg.data == NULL)
			throw sparsestereo::Exception("Unable to open input images!");

		// Load rectification data
		StereoRectification* rectification = NULL;
		if(calibFile != NULL)
			rectification = new StereoRectification(CalibrationResult(calibFile));
		
		// The stereo matcher. SSE Optimized implementation is only available for a 5x5 window
		SparseStereo<CensusWindow<5>, short> stereo(maxDisp, 1, uniqueness,
			rectification, false, false, leftRightStep);
		
		// Feature detectors for left and right image
		FeatureDetector* leftFeatureDetector = new ExtendedFAST(true, minThreshold, adaptivity, false, 2);
		FeatureDetector* rightFeatureDetector = new ExtendedFAST(false, minThreshold, adaptivity, false, 2);

		ptime lastTime = microsec_clock::local_time();
		vector<SparseMatch> correspondences;
		
		// Objects for storing final and intermediate results
		cv::Mat_<char> charLeft(leftImg.rows, leftImg.cols),
			charRight(rightImg.rows, rightImg.cols);
		Mat_<unsigned int> censusLeft(leftImg.rows, leftImg.cols),
			censusRight(rightImg.rows, rightImg.cols);
		vector<KeyPoint> keypointsLeft, keypointsRight;
		
		// For performance evaluation we do the stereo matching 100 times
		for(int i=0; i< 100; i++) {
			// Featuredetection. This part can be parallelized with OMP
			#pragma omp parallel sections default(shared) num_threads(2)
			{
				#pragma omp section
				{
					ImageConversion::unsignedToSigned(leftImg, &charLeft);
					Census::transform5x5(charLeft, &censusLeft);
					keypointsLeft.clear();
					leftFeatureDetector->detect(leftImg, keypointsLeft);
				}
				#pragma omp section
				{
					ImageConversion::unsignedToSigned(rightImg, &charRight);
					Census::transform5x5(charRight, &censusRight);
					keypointsRight.clear();
					rightFeatureDetector->detect(rightImg, keypointsRight);
				}
			}
				
			// Stereo matching. Not parallelized (overhead too large)
			correspondences.clear();
			stereo.match(censusLeft, censusRight, keypointsLeft, keypointsRight, &correspondences);
		}
		
		// Print statistics
		time_duration elapsed = (microsec_clock::local_time() - lastTime);
		cout << "Time for 100x stereo matching: " << elapsed.total_microseconds()/1.0e6 << "s" << endl
			<< "Features detected in left image: " << keypointsLeft.size() << endl
			<< "Features detected in right image: " << keypointsRight.size() << endl
			<< "Percentage of matched features: " << (100.0 * correspondences.size() / keypointsLeft.size()) << "%" << endl;

		// Highlight matches as colored boxes
		Mat_<Vec3b> screen(leftImg.rows, leftImg.cols);
		cvtColor(leftImg, screen, CV_GRAY2BGR);
		
		for(int i=0; i<(int)correspondences.size(); i++) {
			double scaledDisp = (double)correspondences[i].disparity() / maxDisp;
			Vec3b color;
			if(scaledDisp > 0.5)
				color = Vec3b(0, (1 - scaledDisp)*512, 255);
			else color = Vec3b(0, 255, scaledDisp*512);
			
			rectangle(screen, correspondences[i].imgLeft->pt - Point2f(2,2),
				correspondences[i].imgLeft->pt + Point2f(2, 2), 
				(Scalar) color, CV_FILLED);
		}

		// Display image and wait
		namedWindow("Stereo");
		imshow("Stereo", screen);
		waitKey();
		
		// Clean up
		delete leftFeatureDetector;
		delete rightFeatureDetector;
		if(rectification != NULL)
			delete rectification;
			
		return 0;
	}
	catch (const std::exception& e) {
		cerr << "Fatal exception: " << e.what();
		return 1;
	}
}
