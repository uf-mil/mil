#include <iostream>
#include <iterator>

#include <bvt_sdk.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class BlueViewSonar
{
public:
  /// Represents the two ways to connect to a sonar, either from a live device or a .son file
  enum ConnectionType
  {
    FILE,
    DEVICE
  };
  BlueViewSonar();
  ~BlueViewSonar();

  /**  Connects to sonar, called by constructor but can be recalled to retry connection
   *   \param params ConnectionType specific paramater, IP/Domain of sonar or path to .son file
   *   \param head_id For systems with multiple heads, which head to connect to
   */
  void init(ConnectionType type, const std::string& params, int head_id = 0);

  /**
   * Retrieves a new ping from file or sonar, must be called before other processing functions
   * \return true if ping successful, false if error (like end of file)
   */
  bool getNextPing();

  /// \return True if a ping has been retrieved
  bool hasPing() const;

  /// \return True if an image has been generated
  bool hasImage() const;

  /** Generates a mag image from most recent ping, call before getting images
   *  \pre getNextPing has been called and returned True
   */
  void generateImage();

  /** Fills an opencv image with the XY mag image from latest ping
   *  \pre generateImage has been called
   *  \param img OpenCV image to fill with mag image
   *  \post img will be a CV_16U OpenCV Mat with mag image
   */
  void getGrayscaleImage(cv::Mat& img);

  /** Uses loaded colar mapper to create a color image of latest ping
   *  \pre generateImage has been called
   *  \pre loadColorMapper has been called
   *  \param img OpenCV Mat to fil with color XY ping image
   *  \post img is a CV_8UC4 mat with color image of latest ping
   */
  void getColorImage(cv::Mat& img);

  /** Loads a blueview color map for use in color image generation
   *  \param file Path to blueview .cmap file
   */
  void loadColorMapper(const std::string& file);

  /** Fills bearings with location of range in degrees and ranges with corresponding distance in meters
   *  \pre getNextPing has been called and returned True
   */
  void getRanges(std::vector<float>& bearings, std::vector<float>& ranges, std::vector<uint16_t>& intensities);

  /// Return a reference to blueview sdk head object for configuration like setting range
  /// Note: Must call updateHead after making changes for all changes to be used
  BVTSDK::Head& getHead();

  /// Update internal members to changes made after call to getHead
  void updateHead();

  /// Returns number of pings in file, -1 if connected to device
  int getPingCount();

  /// Sets minimum intensity value from blueview to be included in call to getRanges
  void SetRangeProfileMinIntensity(uint16_t thresh);

  /** Sets threshold of dynamic noise to filter out for images and ranges
   *  \param thresh float between 0.0 and 1.0, where 0.0 thresholds no noise and 1.0 threshold all noise
   */
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
