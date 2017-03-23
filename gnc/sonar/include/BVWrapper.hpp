#include <iostream>
#include <iterator>

#include <bvt_sdk.h>

#include <opencv2/core/core.hpp> 
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/package.h>

/*
const int BW8 = 0;
const int BW16 = 1;
const int COLOR = 2;
const int DEVICE = 3;
const int FILE = 4;
*/
class BlueViewSonar {
	public:
		enum ConnectionType
		{
			FILE,
			DEVICE
		};
		BlueViewSonar();
		~BlueViewSonar();

		void getNextPing();                   // Must be called before any data proccessing (image or range profile) calls
		bool hasPing() const;
		bool hasImage() const;

		void generateImage();                 // Internally generates image from latest ping, must be called before running 
		void getGrayscaleImage(cv::Mat& img); // CV_16U img to fill with grayscale intensity
		void getColorImage(cv::Mat& img);     // CV_8UC4 to fill with mapped colors from loadColorMapper
		
		// Fills bearings with location of range in degrees and ranges with corresponding distance in meters
		void getRanges(std::vector<float>& bearings, std::vector<float>& ranges, std::vector<uint16_t>& intensities);

		BVTSDK::Head& getHead();
		void loadColorMapper(const std::string& file);
		int getPingCount();
		void setStopRange(float val);
		void init(ConnectionType type, const std::string& params, int head_id = 0);
	private:
		ConnectionType connection_type;
		bool mapColors(BVTMagImage img, BVTColorImage& cimg);
		bool getCVImages(BVTMagImage img, cv::Mat& image8, cv::Mat& image16, cv::Mat& image84);
		BVTSDK::Sonar sonar;
		BVTSDK::ColorMapper color_mapper;
		BVTSDK::MagImage mag_img;
		BVTSDK::Head head;
		BVTSDK::Ping latest_ping;
		BVTSDK::ImageGenerator image_generator;
		int cur_ping;
		bool has_ping, has_image, has_color_map;
		
		void SetRange(float lower, float upper);
		/*
		BVTHead head = NULL;
		BVTSonar son = NULL;
		BVTPing ping = NULL;

		BVTColorMapper mapper = NULL;

		unsigned short int* bitBuffer = NULL;
		BVTColorImage cimg = NULL;
		BVTMagImage img = NULL;


		const char* param = "";
		const int lowerRange = -1;
		const int upperRange = -1;
		*/

};
