#include "BVWrapper.hpp"

BlueViewSonar::BlueViewSonar()
  : head_(NULL), latest_ping_(NULL), mag_img_(NULL), has_ping_(false), has_image_(false), cur_ping_(0)
{
}
bool BlueViewSonar::hasImage() const
{
  return has_image_;
}
void BlueViewSonar::init(ConnectionType type, const std::string& params, int head_id)
{
  connection_type_ = type;
  // Open the sonar as either a recorded data file or live device
  if (connection_type_ == ConnectionType::FILE)
    sonar_.Open("FILE", params);
  else
    sonar_.Open("NET", params);  // default ip address
  head_ = sonar_.GetHead(head_id);
  image_generator_.SetHead(head_);
}
BVTSDK::Head& BlueViewSonar::getHead()
{
  return head_;
}
BlueViewSonar::~BlueViewSonar()
{
}
bool BlueViewSonar::hasPing() const
{
  return has_ping_;
}
void BlueViewSonar::SetRangeProfileMinIntensity(uint16_t thresh)
{
  image_generator_.SetRangeProfileIntensityThreshold(thresh);
}
bool BlueViewSonar::getNextPing()
{
  image_generator_.SetHead(head_);
  cur_ping_++;
  int ping_num = -1;
  if (connection_type_ == ConnectionType::FILE)
  {
    if (cur_ping_ + 1 > head_.GetPingCount()) return false;
    ping_num = cur_ping_;
  }
  latest_ping_ = head_.GetPing(ping_num);
  has_ping_ = true;
  return true;
}

int BlueViewSonar::getPingCount()
{
  return head_.GetPingCount();
}
void BlueViewSonar::generateImage()
{
  if (!hasPing())
    throw std::runtime_error("Cannot generate image before calling getNextPing");
  mag_img_ = image_generator_.GetImageXY(latest_ping_);
  has_image_ = true;
}
void BlueViewSonar::getGrayscaleImage(cv::Mat& img)
{
  if (!hasImage())
    throw std::runtime_error("Cannot get grayscale image before calling generateImages()");
  int height = mag_img_.GetHeight();
  int width = mag_img_.GetWidth();
  size_t buffer_len = width * height;
  uint16_t buffer[buffer_len];
  mag_img_.CopyBits(buffer, buffer_len);
  img = cv::Mat(height, width, CV_16U, buffer);
}
void BlueViewSonar::loadColorMapper(const std::string& file)
{
  color_mapper_.Load(file);
  has_color_map_ = true;
}
void BlueViewSonar::getColorImage(cv::Mat& img)
{
  if (!hasImage())
    throw std::runtime_error("Cannot get color image before calling generateImages()");
  if (!has_color_map_)
    throw std::runtime_error("Cannot get color image before calling loadColorMapper");
  BVTSDK::ColorImage bv_img = color_mapper_.MapImage(mag_img_);
  int height = mag_img_.GetHeight();
  int width = mag_img_.GetWidth();
  size_t buffer_len = height * width;
  uint32_t buffer[buffer_len];
  bv_img.CopyBits(buffer, buffer_len);
  img = cv::Mat(height, width, CV_8UC4, buffer);
}
void BlueViewSonar::getRanges(std::vector<float>& bearings, std::vector<float>& ranges,
                              std::vector<uint16_t>& intensities)
{
  if (!hasPing())
    throw std::runtime_error("Cannot generate range profile before calling getNextPing");
  BVTSDK::RangeProfile range_profile = image_generator_.GetRangeProfile(latest_ping_);

  // Handles very annoying bug in library with older sonar files that don't support range profiles
  if (range_profile.Handle() == nullptr)
  {
    throw std::runtime_error("Range profile can not be generated. Perhaps this is an old sonar / file. Please disable "
                             "raw data");
  }
  size_t count = range_profile.GetCount();
  // Resize vectors to number of hits
  bearings.resize(count);
  ranges.resize(count);
  intensities.resize(count);

  // Fill vectors with bearing, range, and intensity values
  range_profile.CopyBearingValues(&bearings[0], count);
  range_profile.CopyRangeValues(&ranges[0], count);
  range_profile.CopyIntensityValues(&intensities[0], count);
}
