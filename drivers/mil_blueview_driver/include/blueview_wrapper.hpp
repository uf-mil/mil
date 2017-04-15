#include <iostream>
#include <iterator>

#include <bvt_sdk.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class BlueViewSonar
{
public:
  enum ConnectionType
  {
    FILE,
    DEVICE
  };
  BlueViewSonar();
  ~BlueViewSonar();

  // Connects to sonar, called by constructor but can be recalled to retry connection
  void init(ConnectionType type, const std::string& params, int head_id = 0);

  bool getNextPing();  // Must be called before any data proccessing (image or range profile) calls
  bool hasPing() const;
  bool hasImage() const;

  void generateImage();                  // Internally generates image from latest ping, must be called before running
  void getGrayscaleImage(cv::Mat& img);  // CV_16U img to fill with grayscale intensity
  void getColorImage(cv::Mat& img);      // CV_8UC4 to fill with mapped colors from loadColorMapper
  void loadColorMapper(const std::string& file);

  // Fills bearings with location of range in degrees and ranges with corresponding distance in meters
  void getRanges(std::vector<float>& bearings, std::vector<float>& ranges, std::vector<uint16_t>& intensities);

  // Return a reference to the head for configuration like setting range
  BVTSDK::Head& getHead();
  void updateHead(); // Must call after changes to head from getHead()
  int getPingCount();

  void SetRangeProfileMinIntensity(uint16_t thresh);
  void SetNoiseThreshold(float thresh);

private:
  ConnectionType connection_type_;
  BVTSDK::Sonar sonar_;
  BVTSDK::ColorMapper color_mapper_;
  BVTSDK::MagImage mag_img_;
  BVTSDK::Head head_;
  BVTSDK::Ping latest_ping_;
  BVTSDK::ImageGenerator image_generator_;
  int cur_ping_;
  bool has_ping_, has_image_, has_color_map_, has_init_;
};
