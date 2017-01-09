#include <iostream>
#include <iterator>

#include <bvt_sdk.h>

#include <opencv2/core/core.hpp> 
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/package.h>

namespace bv{
	const int BW8 = 0;
	const int BW16 = 1;
	const int COLOR = 2;
	const int DEVICE = 3;
	const int FILE = 4;
	class BVWrapper{
		private:

			bool mapColors(BVTMagImage img, BVTColorImage& cimg);
			bool getCVImages(BVTMagImage img, cv::Mat& image8, cv::Mat& image16, cv::Mat& image84);
			void initializeSonar();

			int dataSource = 0;
			BVTHead head = NULL;
			BVTSonar son = NULL;
			int currentPing = 0;
			BVTColorMapper mapper = NULL;

			unsigned short int* bitBuffer = NULL;
			BVTColorImage cimg = NULL;
			BVTMagImage img = NULL;

			const char* param = "";
			const int lowerRange = -1;
			const int upperRange = -1;

		public:

			BVWrapper(const int type, const char* param, const int lowerRange=1, const int upperRange=40);
			~BVWrapper();
			bool getNextPing(cv::Mat& image8, cv::Mat& image16, cv::Mat& image84);
			int getPingCount();
			void setStopRange(float val);
			void reinitializeSonar();

	};
}