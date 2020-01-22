/*
 * Author: Konstantin Schauwecker
 * Year:   2012
 */

#include "sparserectification.h"
#include "stereorectification.h"

namespace sparsestereo {
	using namespace std;
	using namespace cv;

	SparseRectification::SparseRectification(bool subpixelFeatures, StereoRectification* rect)
		: subpixelFeatures(subpixelFeatures), rect(rect) {
	}
	
	void SparseRectification::rectify(const vector<KeyPoint>& inLeft, const vector<KeyPoint>& inRight, vector<RectifiedFeature>* outLeft, vector<RectifiedFeature>* outRight) {
	
		outLeft->resize(inLeft.size());
		outRight->resize(inRight.size());
		
		//#pragma omp parallel sections default(shared) num_threads(2)
		{
			//#pragma omp section
			{
				if(rect != NULL) {
					// Rectify left features
					if(subpixelFeatures)
						for(int i=0; i<(int)inLeft.size(); i++) {
							(*outLeft)[i].imgPoint = &inLeft[i];
							(*outLeft)[i].rectPoint = rect->rectifyLeftPoint(inLeft[i].pt);
						}
					else for(int i=0; i<(int)inLeft.size(); i++) {
						(*outLeft)[i].imgPoint = &inLeft[i];
						(*outLeft)[i].rectPoint = rect->rectifyLeftPoint(cv::Point2i(inLeft[i].pt.x, inLeft[i].pt.y));
					}
				} else
					// Copy unrectified
					for(int i=0; i<(int)inLeft.size(); i++) {
						(*outLeft)[i].imgPoint = &inLeft[i];
						(*outLeft)[i].rectPoint = inLeft[i].pt;
					}
				
				// Sort features
				sort(outLeft->begin(), outLeft->end(), featureSortComp);
			}
			
			//#pragma omp section
			{
				if(rect != NULL) {
					// Rectify right features
					if(subpixelFeatures)
						for(int i=0; i<(int)inRight.size(); i++) {
							(*outRight)[i].imgPoint = &inRight[i];
							(*outRight)[i].rectPoint = rect->rectifyRightPoint(inRight[i].pt);
						}
					else  for(int i=0; i<(int)inRight.size(); i++) {
						(*outRight)[i].imgPoint = &inRight[i];
						(*outRight)[i].rectPoint = rect->rectifyRightPoint(cv::Point2i(inRight[i].pt.x, inRight[i].pt.y));
					} 
				} else
					// Copy unrectified
					for(int i=0; i<(int)inRight.size(); i++) {
						(*outRight)[i].imgPoint = &inRight[i];
						(*outRight)[i].rectPoint = inRight[i].pt;
					}
				
				// Sort features
				sort(outRight->begin(), outRight->end(), featureSortComp);
			}
		}
	}
	
	void SparseRectification::precomputeEpilinesStart(int imageWidth, int imageHeight, Mat_<short int>* dst) {
		Epiline::setMaxEpilineLength(imageWidth);
		(*dst) = cv::Mat_<short>(imageHeight, imageWidth, (short)0);
		
		for(int y=0; y<imageHeight; y++)
			for(int x=0; x<imageWidth; x++)
				(*dst)(y, x) = estimateDistortedInfiniteLeftX(y, x);
	}
	
	inline float SparseRectification::estimateDistortedInfiniteLeftX(int leftImgY, int rightRectX) {
		if(rect != NULL) {
			// Iterative approximation
			const int maxIterations = 10;
			const float epsilon = 0.2;
			
			float d = 1;
			float x1 = max(0, min(rect->getCalibrationResult().imageSize.width-1, rightRectX));
				
			for(int i=0; i<maxIterations && fabs(d) > epsilon; i++) {
				Epiline epiline = rect->getLeftEpiline(cv::Point2f(x1,  leftImgY));
				if(!epiline.isValid())
					break; // Lets quit early if we can't find a valid epiline
				float y1 = epiline.at(int(x1+0.5));
				
				y1 = max(0.0F, min((float)rect->getCalibrationResult().imageSize.height-1, y1));
				
				cv::Point2f guess = rect->rectifyLeftPoint(cv::Point2f(x1, y1));
				float d = guess.x - rightRectX;
			
				x1=max(0.0F, min((float)rect->getCalibrationResult().imageSize.width-1, x1-d));
			}
			return x1;
		} else {
			// We don't have to guess, we know it!
			return rightRectX;
		}
	}
}
