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

sensor_msgs::ImageConstPtr left_most_recent {nullptr},  right_most_recent {nullptr};
sensor_msgs::CameraInfoConstPtr left_most_recent_info, right_most_recent_info;
image_transport::CameraSubscriber left_image_sub, right_image_sub;
image_geometry::PinholeCameraModel left_cam_model, right_cam_model;
Matx34d left_P, right_P;  // camera projection matrices
string tf_frame_l, tf_frame_r; // tf frames
vector<Eigen::Vector3d> stereo_point_cloud;
ros::Publisher pcl_pub;
boost::mutex left_mtx, right_mtx;
bool active = false;
ros::ServiceServer detection_switch;
string activation_srv_name {"stereo/activation_srv"};

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
    int max_features_default = 20;
    int feature_block_size_default = 11;
    float feature_min_distance_default = 20.0;
    int img_height = 482;
    int img_width = 684;

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

    vector<SparseMatch> correspondences;

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
        cv::Mat_<unsigned char> raw_left, raw_right, leftImg, rightImg;

        // Thread Safety
        left_mtx.lock();
        right_mtx.lock();
        {
          // Get Frame Message data
          namespace cvb = cv_bridge;
          cout << "left_most_recent: " << left_most_recent << endl;
          cout << "right_most_recent: " << right_most_recent << endl;
          input_bridge = cvb::toCvCopy(left_most_recent, sensor_msgs::image_encodings::MONO8);
          raw_left = input_bridge->image;
          cout << "raw_left_size:"  << raw_left.size() << endl;
          input_bridge = cvb::toCvCopy(right_most_recent, sensor_msgs::image_encodings::MONO8);
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
        cv::Mat_<char> charLeft(leftImg.rows, leftImg.cols),
          charRight(rightImg.rows, rightImg.cols);
        Mat_<unsigned int> censusLeft(leftImg.rows, leftImg.cols),
          censusRight(rightImg.rows, rightImg.cols);
        vector<KeyPoint> keypointsLeft, keypointsRight;
        
        
        // Featuredetection. This part can be parallelized with OMP
        // TODO: split color image channels and get correspondences in each
        ptime lastTime = microsec_clock::local_time();
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
        
        // Print statistics
        time_duration elapsed = (microsec_clock::local_time() - lastTime);
        cout << "Time for stereo matching: " << elapsed.total_microseconds()/1.0e6 << "s" << endl
          << "Features detected in left image: " << keypointsLeft.size() << endl
          << "Features detected in right image: " << keypointsRight.size() << endl
          << "Percentage of matched features: " << (100.0 * correspondences.size() / keypointsLeft.size()) << "%" << endl;

        // Highlight matches as colored boxes
        Mat_<Vec3b> screen(leftImg.rows, leftImg.cols*2);
        Mat_<Vec3b> screen_l(leftImg.rows, leftImg.cols);
        Mat_<Vec3b> screen_r(rightImg.rows, rightImg.cols);
        Rect screen_left_roi {Point(0,0), leftImg.size()};
        Rect screen_right_roi {Point(leftImg.cols,0), leftImg.size()};

        cvtColor(leftImg, screen_l, CV_GRAY2BGR);
        cvtColor(rightImg, screen_r, CV_GRAY2BGR);
        
        for(int i=0; i<(int)correspondences.size(); i++) {
          // Visualize Matches
          double scaledDisp = (double)correspondences[i].disparity() / maxDisp;
          Scalar color {rand() % 256, rand() % 256,rand() % 256};
          Point2f pt_L = correspondences[i].imgLeft->pt;
          Point2f pt_R = correspondences[i].imgRight->pt;
          rectangle(screen_l, pt_L - Point2f(2,2), pt_L + Point2f(2, 2), 
                    color, CV_FILLED);
          rectangle(screen_r, pt_R - Point2f(2,2), pt_R + Point2f(2, 2), 
                    color, CV_FILLED);

          // Triangulate matches to reconstruct 3D point
          Matx31d pt_L_hom(pt_L.x, pt_L.y, 1);
          Matx31d pt_R_hom(pt_R.x, pt_R.y, 1);
          cout << "ptL: " << pt_L << " ptR: " << pt_R << endl;
          Mat X_hom = nav::triangulate_Linear_LS(Mat(left_P),
                                                 Mat(right_P),
                                                 Mat(pt_L_hom), Mat(pt_R_hom));
          X_hom = X_hom / X_hom.at<double>(3, 0);
          Eigen::Vector3d pt_3D;
          pt_3D << 
            X_hom.at<double>(0, 0), X_hom.at<double>(1, 0), X_hom.at<double>(2, 0);
          stereo_point_cloud.push_back(pt_3D);
        }

        cout << "3D point: " << stereo_point_cloud[0] << endl;
        
        // Display image and wait
        // imshow("left", raw_left); imshow("right", raw_right); waitKey(0);
        screen_l.copyTo(screen(screen_left_roi));
        screen_r.copyTo(screen(screen_right_roi));
        namedWindow("Stereo");
        imshow("Stereo", screen);
        waitKey(50);
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
