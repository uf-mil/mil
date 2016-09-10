/*
 * Author: David Soto
 * Year:   2016
 */
 
// this node uses a stereo camera rig to generate a point cloud of the environment.
// It uses the exFAST detector to generate a sparse point cloud.
// See this for more info: http://www.ra.cs.uni-tuebingen.de/software/sparsestereo/welcome_e.html

#include <navigator_vision_lib/cv_tools.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <vector>
#include <iostream>
#include <iomanip>
#include <sparsestereo/exception.h>
#include <sparsestereo/extendedfast.h>
#include <sparsestereo/stereorectification.h>
#include <sparsestereo/sparsestereo-inl.h>
#include <sparsestereo/census-inl.h>
#include <sparsestereo/imageconversion.h>
#include <sparsestereo/censuswindow.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
// #include <tf/transform_listener.h>
#include <navigator_msgs/ActivationSwitch.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
 #include <sensor_msgs/point_cloud2_iterator.h>

void left_image_callback(
    const sensor_msgs::ImageConstPtr &image_msg_ptr,
    const sensor_msgs::CameraInfoConstPtr &info_msg_ptr);
void right_image_callback(
    const sensor_msgs::ImageConstPtr &image_msg_ptr,
    const sensor_msgs::CameraInfoConstPtr &info_msg_ptr);
bool detection_activation_switch(
    navigator_msgs::ActivationSwitch::Request &req,
    navigator_msgs::ActivationSwitch::Response &resp);

using namespace std;
using namespace cv;
using namespace sparsestereo;
using namespace boost;
using namespace boost::posix_time;
using ros::param::param;

struct XYZRGB{

  float x, y, z;
  uint8_t r, g, b;
  
  bool operator==(const XYZRGB &other) const{
    if(this->x == other.x && this->y == other.y && this->z == other.z){
      return true;
    }
    else return false;
  }

  bool operator<(const XYZRGB &other) const{
    if(this->x > other.x) return false;
    else if(this->y > other.y) return false;
    else if(this->z >= other.z) return false;
    else return true;
  }
};

sensor_msgs::ImageConstPtr left_most_recent {nullptr},  right_most_recent {nullptr};
sensor_msgs::CameraInfoConstPtr left_most_recent_info, right_most_recent_info;
image_transport::CameraSubscriber left_image_sub, right_image_sub;
image_geometry::PinholeCameraModel left_cam_model, right_cam_model;
Matx34d left_P, right_P;  // camera projection matrices
string tf_frame_l, tf_frame_r; // tf frames
vector<XYZRGB> stereo_point_cloud;
ros::Publisher pcl_pub;
boost::mutex left_mtx, right_mtx;
bool active = false;
ros::ServiceServer detection_switch;
string activation_srv_name {"stereo/activation_srv"};
sensor_msgs::PointCloud2 pt_cloud_msg;
std_msgs::Header header;
int pt_cloud_seq {0};



int main(int argc, char** argv) {
namespace fs = boost::filesystem;
  try {

    stringstream log_msg;
    log_msg << "\nInitializing Sparse Stereo Point Cloud Driver:\n";
    int tab_sz = 4;

    // globals
    ros::init(argc, argv, "stereo_point_cloud_driver");
    ros::NodeHandle nh;
    image_transport::ImageTransport img_trans {nh};
    image_geometry::PinholeCameraModel left_cam_model, right_cam_model;
    cv_bridge::CvImagePtr input_bridge;

    // Default parameters
    string img_topic_left_default = "/stereo/left/image_rect_color/";
    string img_topic_right_default = "/stereo/right/image_rect_color/";
    string activation_default = "/stereo/activation_switch";
    string dbg_topic_default = "/stereo/dbg_imgs";

    // Advertise activation switch
    detection_switch = nh.advertiseService(
      activation_srv_name, detection_activation_switch);

    // Advertise point cloud topic
    pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("stereo_front_point_cloud", 1);

    // Subscribe to Cameras (image + camera_info)
    cout << "subscribing to cameras" << endl;
    string left = param<string>("/stereo/input_left", img_topic_left_default);
    string right = param<string>("/stereo/input_right", img_topic_right_default);
    cout << "Got topic name params!" << endl;
    left_image_sub = img_trans.subscribeCamera(left, 10, left_image_callback);
    right_image_sub = img_trans.subscribeCamera(right, 10, right_image_callback);
    log_msg << setw(1 * tab_sz) << "" << "Camera Subscriptions:\x1b[37m\n"
      << setw(2 * tab_sz) << "" << "left  = " << left << endl
      << setw(2 * tab_sz) << "" << "right = " << right << "\x1b[0m\n";
    cout << left << ' ' << right << endl;
    ROS_INFO(log_msg.str().c_str());

    // Size adjustment ROI
    /*
      exFAST is currently heavily optimized for VGA size images, extracting a 
      properly sized region of interest is much more efficient than resizing the image
    */

    // Stereo matching parameters
    double uniqueness = 0.7;
    int maxDisp = 70;
    int leftRightStep = 2;
    
    // Feature detection parameters
    double adaptivity = 1.0;
    int minThreshold = 10;

    // Load rectification data
    fs::path calibration_file_path {ros::package::getPath("navigator_perception") + "/calibration.xml"};
    cout << calibration_file_path << endl;
    if ( !boost::filesystem::exists( calibration_file_path ) ) // Check file exists
    {
      cout << "Can't find calibration file!" << std::endl;
      throw "Can't find calibration file!";
    }
    StereoRectification* rectification = NULL;
    rectification = new StereoRectification(CalibrationResult(calibration_file_path.c_str()));

    // The stereo matcher. SSE Optimized implementation is only available for a 5x5 window
    SparseStereo<CensusWindow<5>, short> stereo(maxDisp, 1, uniqueness, rectification, 
                                                false, false, leftRightStep);

    // Feature detectors for left and right image
    FeatureDetector* leftFeatureDetector = new ExtendedFAST(true, minThreshold, adaptivity, false, 2);
    FeatureDetector* rightFeatureDetector = new ExtendedFAST(false, minThreshold, adaptivity, false, 2);

    // Sparse Correspondences
    vector<SparseMatch> correspondencesB;
    vector<SparseMatch> correspondencesG;
    vector<SparseMatch> correspondencesR;

    // Main stereo processing loop
    ros::Rate loop_rate(10);  // process images 10 times per second
    cout << "\x1b[1;31mMain Loop!\x1b[0m" << endl;
    while (ros::ok()) {
      cout << "clock: " << microsec_clock::local_time() << endl;
      cout << " The left Camera Subscriber is connected to " << left_image_sub.getNumPublishers() << " publishers." << endl;
      cout << " The right Camera Subscriber is connected to " << right_image_sub.getNumPublishers() << " publishers." << endl;
      cout << "active=" << (active? "true" : "false") << endl;
      cout  << "left_most_recent=" << left_most_recent << " right_most_recent=" << right_most_recent << endl;
      // Idle if active is not set or valid image pointer pairs have not been retrieved
      if (!active || (left_most_recent == nullptr) || (right_most_recent == nullptr)){
        ros::spinOnce();
        loop_rate.sleep();
        cout << "skipping" << endl;
        continue;
      }
      try {
        // Containers
        // cv::Mat_<unsigned char> raw_left, raw_right, leftImg, rightImg;
        Mat raw_left, raw_right, leftImg, rightImg;

        // Thread Safety
        left_mtx.lock();
        right_mtx.lock();
        {
          // Get Frame Message data
          namespace cvb = cv_bridge;
          cout << "left_most_recent: " << left_most_recent << endl;
          cout << "right_most_recent: " << right_most_recent << endl;
          input_bridge = cvb::toCvCopy(left_most_recent, sensor_msgs::image_encodings::BGR8);
          raw_left = input_bridge->image;
          // imshow("raw_left", raw_left); waitKey(0);
          cout << "raw_left_size:"  << raw_left.size() << endl;
          input_bridge = cvb::toCvCopy(right_most_recent, sensor_msgs::image_encodings::BGR8);
          raw_right = input_bridge->image; 
        }
        left_mtx.unlock();
        right_mtx.unlock();

        // Check images for data and adjust size
        if(raw_left.data == NULL || raw_right.data == NULL)
          throw sparsestereo::Exception("Unable to open input images!");
        leftImg = raw_left;
        rightImg = raw_right;

        // Enforce approximate image synchronization
        double left_stamp = left_most_recent_info->header.stamp.toSec();
        double right_stamp = right_most_recent_info->header.stamp.toSec();
        double sync_error = fabs(left_stamp - right_stamp);
        stringstream sync_msg;
        sync_msg << "Left and right images were not sufficiently synchronized"
                 << "\nsync error: " << sync_error << "s";
        if (sync_error > 0.3) {
          ROS_WARN(sync_msg.str().c_str());
          cout << "skipping" << endl;
          continue;
        }

        // Objects for storing final and intermediate results
        vector<Mat_<unsigned char>> left_channels;
        vector<Mat_<unsigned char>> right_channels;
        split(leftImg, left_channels);
        split(rightImg, right_channels);
        Mat_<unsigned char> left_B, left_G, left_R;
        Mat_<unsigned char> right_B, right_G, right_R;
        left_B = left_channels[0];
        left_G = left_channels[1];
        left_R = left_channels[2];
        right_B = right_channels[0];
        right_G = right_channels[1];
        right_R = right_channels[2];
        int rows = leftImg.rows;
        int cols =leftImg.cols;
        Mat_<char> charLeftB(rows, cols); 
        Mat_<char> charLeftG(rows, cols);
        Mat_<char> charLeftR(rows, cols);
        Mat_<char> charRightB(rows, cols);
        Mat_<char> charRightG(rows, cols);
        Mat_<char> charRightR(rows, cols);
        Mat_<unsigned int> censusLeftB(rows, cols);
        Mat_<unsigned int> censusLeftG(rows, cols);
        Mat_<unsigned int> censusLeftR(rows, cols);
        Mat_<unsigned int> censusRightB(rows, cols);
        Mat_<unsigned int> censusRightG(rows, cols);
        Mat_<unsigned int> censusRightR(rows, cols);
        vector<KeyPoint> keypointsLeftB, keypointsRightB;
        vector<KeyPoint> keypointsLeftG, keypointsRightG;
        vector<KeyPoint> keypointsLeftR, keypointsRightR;
        
        
        // Featuredetection. TODO: parallelize with OMP
        // TODO: split color image channels and get correspondences in each
        ptime lastTime = microsec_clock::local_time();
        ImageConversion::unsignedToSigned(left_B, &charLeftB);
        Census::transform5x5(charLeftB, &censusLeftB);
        keypointsLeftB.clear();
        leftFeatureDetector->detect(left_B, keypointsLeftB);

        ImageConversion::unsignedToSigned(left_G, &charLeftG);
        Census::transform5x5(charLeftG, &censusLeftG);
        keypointsLeftG.clear();
        leftFeatureDetector->detect(left_G, keypointsLeftG);

        ImageConversion::unsignedToSigned(left_R, &charLeftR);
        Census::transform5x5(charLeftR, &censusLeftR);
        keypointsLeftR.clear();
        leftFeatureDetector->detect(left_R, keypointsLeftR);

        ImageConversion::unsignedToSigned(right_B, &charRightB);
        Census::transform5x5(charRightB, &censusRightB);
        keypointsRightB.clear();
        rightFeatureDetector->detect(right_B, keypointsRightB);

        ImageConversion::unsignedToSigned(right_G, &charRightG);
        Census::transform5x5(charRightG, &censusRightG);
        keypointsRightG.clear();
        rightFeatureDetector->detect(right_G, keypointsRightG);

        ImageConversion::unsignedToSigned(right_R, &charRightR);
        Census::transform5x5(charRightR, &censusRightR);
        keypointsRightR.clear();
        rightFeatureDetector->detect(right_R, keypointsRightR);

                
        // Stereo matching and reconstruction
        Point2f pt_L;
        Point2f pt_R;
        Matx31d pt_L_hom;
        Matx31d pt_R_hom;
        Mat X_hom;
        Vec3b color;
        // #pragma omp parallel sections default(shared) num_threads(3)
        {
          // #pragma omp section
          {
            // Calculate correspondences in Blue channel
            correspondencesB.clear();
            stereo.match(censusLeftB, censusRightB, keypointsLeftB, keypointsRightB, &correspondencesB);

            for(int i=0; i<(int)correspondencesB.size(); i++) {

              // Visualize feature points
              rectangle(left_channels[0], pt_L - Point2f(2,2), pt_L + Point2f(2, 2), 
                    rand() % 256, CV_FILLED);
              rectangle(right_channels[0], pt_R - Point2f(2,2), pt_R + Point2f(2, 2), 
                    rand() % 256, CV_FILLED);

              // Triangulate matches to reconstruct 3D point
              pt_L = correspondencesB[i].imgLeft->pt;
              pt_R = correspondencesB[i].imgRight->pt;
              pt_L_hom = Matx31d(pt_L.x, pt_L.y, 1);
              pt_R_hom = Matx31d(pt_R.x, pt_R.y, 1);
              cout << "ptL: " << pt_L << " ptR: " << pt_R << endl;
              X_hom = nav::triangulate_Linear_LS(Mat(left_P), Mat(right_P), Mat(pt_L_hom), Mat(pt_R_hom));
              X_hom = X_hom / X_hom.at<double>(3, 0);

              // Get Color and fill point_cloud element precursor
              XYZRGB point;
              point.x = X_hom.at<double>(0, 0);
              point.y = X_hom.at<double>(1, 0);
              point.z = X_hom.at<double>(2, 0);
              color = leftImg.at<Vec3b>(pt_L);
              point.b = color[0];
              point.g = color[1];
              point.r = color[2];
              stereo_point_cloud.push_back(point);
            }
          }
          // #pragma omp section
          {
            // Calculate correspondences in Green channel
            correspondencesG.clear();
            stereo.match(censusLeftG, censusRightG, keypointsLeftG, keypointsRightG, &correspondencesG);

            for(int i=0; i<(int)correspondencesG.size(); i++) {

              // Visualize feature points
              rectangle(left_channels[1], pt_L - Point2f(2,2), pt_L + Point2f(2, 2), 
                    rand() % 256, CV_FILLED);
              rectangle(right_channels[1], pt_R - Point2f(2,2), pt_R + Point2f(2, 2), 
                    rand() % 256, CV_FILLED);

              // Triangulate matches to reconstruct 3D point
              pt_L = correspondencesG[i].imgLeft->pt;
              pt_R = correspondencesG[i].imgRight->pt;
              pt_L_hom = Matx31d(pt_L.x, pt_L.y, 1);
              pt_R_hom = Matx31d(pt_R.x, pt_R.y, 1);
              cout << "ptL: " << pt_L << " ptR: " << pt_R << endl;
              X_hom = nav::triangulate_Linear_LS(Mat(left_P), Mat(right_P), Mat(pt_L_hom), Mat(pt_R_hom));
              X_hom = X_hom / X_hom.at<double>(3, 0);

              // Get Color and fill point_cloud element precursor
              XYZRGB point;
              point.x = X_hom.at<double>(0, 0);
              point.y = X_hom.at<double>(1, 0);
              point.z = X_hom.at<double>(2, 0);
              color = leftImg.at<Vec3b>(pt_L);
              point.b = color[0];
              point.g = color[1];
              point.r = color[2];
              stereo_point_cloud.push_back(point);
            }
          }
          // #pragma omp section
          {
            // Calculate correspondences in Red channel
            correspondencesR.clear();
            stereo.match(censusLeftR, censusRightR, keypointsLeftR, keypointsRightR, &correspondencesR);

            for(int i=0; i<(int)correspondencesR.size(); i++) {

              // Visualize feature points
              rectangle(left_channels[2], pt_L - Point2f(2,2), pt_L + Point2f(2, 2), 
                    rand() % 256, CV_FILLED);
              rectangle(right_channels[2], pt_R - Point2f(2,2), pt_R + Point2f(2, 2), 
                    rand() % 256, CV_FILLED);

              // Triangulate matches to reconstruct 3D point
              pt_L = correspondencesR[i].imgLeft->pt;
              pt_R = correspondencesR[i].imgRight->pt;
              pt_L_hom = Matx31d(pt_L.x, pt_L.y, 1);
              pt_R_hom = Matx31d(pt_R.x, pt_R.y, 1);
              cout << "ptL: " << pt_L << " ptR: " << pt_R << endl;
              X_hom = nav::triangulate_Linear_LS(Mat(left_P), Mat(right_P), Mat(pt_L_hom), Mat(pt_R_hom));
              X_hom = X_hom / X_hom.at<double>(3, 0);

              // Get Color and fill point_cloud element precursor
              XYZRGB point;
              point.x = X_hom.at<double>(0, 0);
              point.y = X_hom.at<double>(1, 0);
              point.z = X_hom.at<double>(2, 0);
              color = leftImg.at<Vec3b>(pt_L);
              point.b = color[0];
              point.g = color[1];
              point.r = color[2];
              stereo_point_cloud.push_back(point);
            }
          }
        }

        // Delete repeated reconstructions from list
        // set<XYZRGB> s {stereo_point_cloud.begin(), stereo_point_cloud.end()};
        // stereo_point_cloud.assign( s.begin(), s.end() );

        // // Display image 
        // Mat screen = Mat::zeros(Size(leftImg.rows, leftImg.cols*2), CV_8U);
        // Rect left_roi {Point(0,0), leftImg.size()};
        // Rect right_roi {Point(leftImg.cols,0), leftImg.size()};
        // Mat left_screen = screen(left_roi);
        // Mat right_screen = screen(right_roi);
        // merge(left_channels, left_screen);
        // merge(right_channels, right_screen);
        // namedWindow("Stereo");
        // imshow("Stereo", screen);
        // waitKey(30);

        // Print statistics
        time_duration elapsed = (microsec_clock::local_time() - lastTime);
        cout << "Time for stereo matching: " << elapsed.total_microseconds()/1.0e6 << "s" << endl
          << "Features detected in leftB image: " << keypointsLeftB.size() << endl
          << "Features detected in rightB image: " << keypointsRightB.size() << endl
          << "Percentage of matched features: " << (100.0 * correspondencesB.size() / keypointsLeftB.size()) << "%" << endl;
        

        // Populate point cloud message header
        header.seq = pt_cloud_seq++;
        header.stamp = ros::Time::now();
        header.frame_id = tf_frame_l;
        pt_cloud_msg.header = header;

        // Specify point cloud msg meta-data
        pt_cloud_msg.height = 1;
        pt_cloud_msg.width = stereo_point_cloud.size();
        pt_cloud_msg.is_bigendian = false;
        pt_cloud_msg.is_dense = false;

        // Fill point cloud msg
        sensor_msgs::PointCloud2Modifier pcd_modifier(pt_cloud_msg);
        pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
        sensor_msgs::PointCloud2Iterator<float> iter_x(pt_cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(pt_cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(pt_cloud_msg, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(pt_cloud_msg, "b");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(pt_cloud_msg, "g");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(pt_cloud_msg, "r");
        // #pragma omp parallel sections default(shared) num_threads(6)
        {
          // #pragma omp section
          {
            for(size_t i = 0; i < stereo_point_cloud.size(); ++i, ++iter_x)
            {
              *iter_x = stereo_point_cloud[i].x;
            } 
          }
          // #pragma omp section
          {
            for(size_t i = 0; i < stereo_point_cloud.size(); ++i, ++iter_y)
            {
              *iter_y = stereo_point_cloud[i].y;
            } 
          }
          // #pragma omp section
          {
            for(size_t i = 0; i < stereo_point_cloud.size(); ++i, ++iter_z)
            {
              *iter_z = stereo_point_cloud[i].z;
            } 
          }
          // #pragma omp section
          {
            for(size_t i = 0; i < stereo_point_cloud.size(); ++i, ++iter_b)
            {
              *iter_b = stereo_point_cloud[i].b;
            } 
          }
          // #pragma omp section
          {
            for(size_t i = 0; i < stereo_point_cloud.size(); ++i, ++iter_g)
            {
              *iter_g = stereo_point_cloud[i].g;
            } 
          }
          // #pragma omp section
          {
            for(size_t i = 0; i < stereo_point_cloud.size(); ++i, ++iter_r)
            {
              *iter_r = stereo_point_cloud[i].r;
            } 
          }
        }

        pcl_pub.publish(pt_cloud_msg);
        ros::spinOnce();
      }
      catch (const std::exception& e) {
        cerr << "Fatal exception during main loop: " << e.what();
        return -1;
      }
      loop_rate.sleep();
    }  // end main while loop
    return 0;
  } // end large try block
  catch (const std::exception& e) {
    cerr << "Fatal exception during initialization: " << e.what();
    return -1;
  }
}

bool detection_activation_switch(
    navigator_msgs::ActivationSwitch::Request &req,
    navigator_msgs::ActivationSwitch::Response &resp) {
  stringstream ros_log;
  ros_log << "\x1b[1;31mSetting sereo point cloud generation to: \x1b[1;37m"
          << (req.activation_switch ? "on" : "off") << "\x1b[0m";
  ROS_INFO(ros_log.str().c_str());
  active = req.activation_switch;
  resp.success = true;
  return true;
}

void left_image_callback(
    const sensor_msgs::ImageConstPtr &image_msg_ptr,
    const sensor_msgs::CameraInfoConstPtr &info_msg_ptr) {

  // Get ptrs to most recent frame and info (thread safe)
  left_mtx.lock();
  cout << "\x1b[1;31mleft callback!\x1b[0m" << endl;
  left_most_recent = image_msg_ptr;
  left_most_recent_info = info_msg_ptr;
  left_mtx.unlock();

  // Initialize camera parameters
  static bool first_call = true;
  if(first_call){
    left_cam_model.fromCameraInfo(left_most_recent_info);
    left_P = left_cam_model.fullProjectionMatrix();
    tf_frame_l = left_cam_model.tfFrame();
    first_call = false;
  }
}

void right_image_callback(
    const sensor_msgs::ImageConstPtr &image_msg_ptr,
    const sensor_msgs::CameraInfoConstPtr &info_msg_ptr) {

  // Get ptrs to most recent frame and info (thread safe)
  right_mtx.lock();
  cout << "\x1b[1;31mright callback!\x1b[0m" << endl;
  right_most_recent = image_msg_ptr;
  right_most_recent_info = info_msg_ptr;
  right_mtx.unlock();

  // Initialize camera parameters
  static bool first_call = true;
  if(first_call){   
    right_cam_model.fromCameraInfo(right_most_recent_info);
    right_P = right_cam_model.fullProjectionMatrix();
    tf_frame_r = right_cam_model.tfFrame();
    first_call = false;
  }
}
