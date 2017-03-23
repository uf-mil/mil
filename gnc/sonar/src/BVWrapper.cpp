#include "BVWrapper.hpp"

BlueViewSonar::BlueViewSonar() :
	head(NULL),
	latest_ping(NULL),
	mag_img(NULL),
	has_ping(false),
	has_image(false),
	cur_ping(0)
{

}
bool BlueViewSonar::hasImage() const
{
	return has_image;
}
void BlueViewSonar::init(ConnectionType type, const std::string& params, int head_id)
{
	connection_type = type;
	// Open the sonar as either a recorded data file or live device
	if(connection_type == ConnectionType::FILE)
		sonar.Open("FILE", params);
	else
		sonar.Open("NET", params); // default ip address
	// Get the first head, might have to
	head = sonar.GetHead(head_id);
	image_generator.SetHead(head);
}
BVTSDK::Head& BlueViewSonar::getHead()
{
	return head;
}
BlueViewSonar::~BlueViewSonar()
{
	
} 
void BlueViewSonar::SetRange(float lower, float upper)
{
	head.SetRange(lower, upper);
}
bool BlueViewSonar::hasPing() const
{
	return has_ping;
}
void BlueViewSonar::getNextPing()
{
	has_ping = true;
	int ping_num;
	if(connection_type == ConnectionType::FILE) {
		cur_ping++;
		ping_num = cur_ping;
	}
	else
		ping_num = -1;
	latest_ping = head.GetPing(ping_num);
}

int BlueViewSonar::getPingCount()
{
	return head.GetPingCount();
}

void BlueViewSonar::setStopRange(float val)
{
	head.SetStopRange(val);
}
void BlueViewSonar::generateImage()
{
	if (!hasPing()) throw std::runtime_error("Cannot generate image before calling getNextPing");
	has_image = true;
	mag_img = image_generator.GetImageXY(latest_ping);
}
void BlueViewSonar::getGrayscaleImage(cv::Mat& img)
{
	if (!hasImage()) throw std::runtime_error("Cannot get grayscale image before calling generateImages()");
	int height = mag_img.GetHeight();
	int width =  mag_img.GetWidth();
	size_t buffer_len = width * height;
	uint16_t buffer[buffer_len];
	mag_img.CopyBits(buffer, buffer_len);
	img = cv::Mat(height, width, CV_16U, buffer);
}
void BlueViewSonar::loadColorMapper(const std::string& file)
{
	color_mapper.Load(file);
    has_color_map = true;
}
void BlueViewSonar::getColorImage(cv::Mat& img)
{
	if (!hasImage()) throw std::runtime_error("Cannot get color image before calling generateImages()");
    if (!has_color_map) throw std::runtime_error("Cannot get color image before calling loadColorMapper");
	BVTSDK::ColorImage bv_img = color_mapper.MapImage(mag_img);
	int height = mag_img.GetHeight();
	int width = mag_img.GetWidth();
	size_t buffer_len = height*width;
	uint32_t buffer[buffer_len];
	bv_img.CopyBits(buffer, buffer_len);
	img = cv::Mat(height, width, CV_8UC4, buffer);
}
void BlueViewSonar::getRanges(std::vector<float>& bearings, std::vector<float>& ranges, std::vector<uint16_t>& intensities)
{
	BVTSDK::RangeProfile range_profile = image_generator.GetRangeProfile(latest_ping);
	size_t count = range_profile.GetCount();

	// Resize vectors to number of hits
	bearings.resize(count);
	ranges.resize(count);
	intensities.resize(count);

	// Filll vectors with bearing, range, and intensity values
	range_profile.CopyBearingValues(&bearings[0], count);
	range_profile.CopyRangeValues(&ranges[0], count);
	range_profile.CopyIntensityValues(&intensities[0], count);
}
