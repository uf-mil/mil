#include "BVWrapper.h"

namespace bv{
	bool BVWrapper::mapColors(BVTMagImage img, BVTColorImage& cimg){
		if( this->mapper == NULL )
		{
			printf("BVTColorMapper_Create: failed\n");
			return false;
		}
		
		// Load the bone colormap
		int ret = BVTColorMapper_Load(this->mapper, (ros::package::getPath("sonar") + "/res/jet.cmap").c_str());
		if( ret != 0 )
		{
			printf("BVTColorMapper_Load: ret=%d\n", ret);
			return false;
		}

		ret = BVTColorMapper_MapImage(this->mapper, img, &cimg);
		if( ret != 0 )
		{
			printf("BVTColorMapper_MapImage: ret=%d\n", ret);
			return 1;
		}
		return true;

	}

	bool BVWrapper::getCVImages(BVTMagImage img, cv::Mat& image8, cv::Mat& image16, cv::Mat& image84){
		int height;
		BVTMagImage_GetHeight(img, &height);
		int width;
		BVTMagImage_GetWidth(img, &width);
		const int arr_size = width * height;

		BVTMagImage_GetBits(img, &this->bitBuffer);
		unsigned short bitBufferCopy[arr_size];

		// Copy the array over so that is now 'owned' by our program
		std::copy(this->bitBuffer, this->bitBuffer + arr_size, bitBufferCopy);

		cv::Mat gray_img(height, width, CV_16U, (void*)bitBufferCopy);
		image16 = gray_img;

		double minVal;
		double maxVal; 
		cv::Point minLoc, maxLoc; 
		minMaxLoc( gray_img, &minVal, &maxVal, &minLoc, &maxLoc );
		double min = 0;
		double max = maxVal;
		cv::minMaxIdx(gray_img, &min, &max);
		cv::Mat gray_img_norm;
		cv::convertScaleAbs(gray_img, gray_img_norm, 255 / max);
		image8 = gray_img_norm;


		if(!mapColors(img, this->cimg)){
			return false;
		}
		unsigned int* bits;
		unsigned int bitsCopy[arr_size];
		BVTColorImage_GetBits(cimg, &bits);
		std::copy(bits, bits + arr_size, bitsCopy);
		cv::Mat color_img(height, width, CV_8UC4, (void*)bitsCopy);
		image84 = color_img;

		BVTColorImage_Destroy(this->cimg);
        BVTMagImage_Destroy(this->img);
		return true;

	}

	void BVWrapper::initializeSonar(){
		this->son = BVTSonar_Create();
		if( this->son == NULL )
			throw "BVTSonar_Create: failed\n";
		int ret;
		if(this->dataSource == FILE){
			// Open the sonar
			ret = BVTSonar_Open(this->son, "FILE", this->param);
			if( ret != 0 )
				throw "BVTSonar_Open: failed";
		}
		else if(this->dataSource == DEVICE){
			ret = BVTSonar_Open(this->son, "NET", this->param); // default ip address
			if( ret != 0 )
				throw "BVTSonar_Open: failed";
		}

		// Get the first head
		ret = BVTSonar_GetHead(this->son, 0, &this->head);
		if( ret != 0 ){
			// some sonars start at head 1 instead of zero...
	        ret = BVTSonar_GetHead(this->son, 1, &this->head);
	        if( ret != 0 )
				throw "BVTSonar_GetHead: failed";
		}

		BVTHead_SetRange(this->head, this->lowerRange, this->upperRange);

		int speed = -1;
		BVTHead_GetSoundSpeed (this->head, &speed);
		std::cout << "Sound Speed: " << speed << std::endl; 
		float gain = -1.0;
		BVTHead_GetGainAdjustment(this->head, &gain);
		std::cout << "Gain: " << gain << std::endl;
		int flags = -1;
		BVTHead_GetImageFilterFlags (this->head, &flags);
		std::cout << "Image Filter Flags: " << flags << std::endl;
		int method;
		BVTHead_GetImageProcessingMethod (this->head, &method);
		std::cout << "Sound Speed: " << method << std::endl;
	}

	void BVWrapper::reinitializeSonar(){
		BVTSonar_Destroy(this->son);
		initializeSonar();
	}

	BVWrapper::BVWrapper(const int type, const char* param, const int lowerRange, const int upperRange):
		dataSource(type), param(param), lowerRange(lowerRange), upperRange(upperRange){
		this->mapper = BVTColorMapper_Create();

		initializeSonar();
	}

	BVWrapper::~BVWrapper(){
		BVTSonar_Destroy(this->son);
		BVTColorMapper_Destroy(this->mapper);
	} 

	bool BVWrapper::getNextPing(cv::Mat& image8, cv::Mat& image16, cv::Mat& image84){
		int ping_num = 0;
		if(this->dataSource == FILE){
			ping_num = this->currentPing++;
		}
		else if(this->dataSource == DEVICE){
			ping_num = -1;
		}
		else{
			std::cout << "The datasource type was incorrectly set" << std::endl;
			return false;
		}

		BVTPing ping = NULL;
		int ret = BVTHead_GetPing(head, ping_num, &ping);
		if( ret != 0 )
		{
			printf("Could not get the next ping: ret=%d\n", ret);
			BVTPing_Destroy(ping);
			return false;
		}
		
		// Generate an image from the ping
		ret = BVTPing_GetImage(ping, &this->img);
		BVTPing_Destroy(ping);
		if( ret != 0 )
		{
			printf("Could not get the image from the ping: ret=%d\n", ret);
			return false;
		}
		bool val = getCVImages(img, image8, image16, image84);
		return val;
	}

	int BVWrapper::getPingCount(){
		int pings = -1;
		BVTHead_GetPingCount(this->head, &pings);
		return pings;
	}

	void BVWrapper::setStopRange(float val){
		BVTHead_SetStopRange(this->head, val);
	}
}