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
#include <Eigen/Dense>
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
#include <exception>
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
#include <tf/transform_listener.h>
#include <std_srvs/SetBool.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <navigator_vision_lib/point_cloud_algorithms.hpp>

void left_image_callback(
    const sensor_msgs::ImageConstPtr &image_msg_ptr,
    const sensor_msgs::CameraInfoConstPtr &info_msg_ptr);
void right_image_callback(
    const sensor_msgs::ImageConstPtr &image_msg_ptr,
    const sensor_msgs::CameraInfoConstPtr &info_msg_ptr);
void cloud_callback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
bool detection_activation_switch(
    std_srvs::SetBool::Request &req,
    std_srvs::SetBool::Response &resp);

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

// Image storage
sensor_msgs::ImageConstPtr left_most_recent,  right_most_recent;
sensor_msgs::CameraInfoConstPtr left_most_recent_info, right_most_recent_info;

// Stereo subscribing
image_transport::CameraSubscriber left_image_sub, right_image_sub;
image_geometry::PinholeCameraModel left_cam_model, right_cam_model;

// Velodyne subscribing
ros::Subscriber pcd_sub;

// Camera projection matrices
Matx34d left_P, right_P;

// TF
// tf::TransformListener tf_listener;
string tf_frame_stereo_l {"/stereo_left_cam"};
string tf_frame_stereo_r {"/stereo_right_cam"};
string tf_frame_velodyne {"/velodyne"};
tf::StampedTransform stereoL_tf_velodyne;

// Point clouds
vector<XYZRGB> stereo_point_cloud;
// sensor_msgs::PointCloud2 pt_cloud_msg;
std_msgs::Header header;
int pt_cloud_seq {0};

// Point cloud publisher
ros::Publisher pcl_pub;

// Thread safety
boost::mutex left_mtx, right_mtx;

// Control
bool active = false;
ros::ServiceServer detection_switch;


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
    string activation_srv_name {"stereo/activation_srv"};
    nav::PcdColorizer vel_colorizer{nh, "/velodyne_points", "/colored_velodyne_cloud", "/stereo/left/image_rect_color", "/stereo_left_cam"};

    // Default parameters
    string img_topic_left_default = "/stereo/left/image_raw/";
    string img_topic_right_default = "/stereo/right/image_raw/";
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

    // Subscribe to Velodyne scan
    pcd_sub = nh.subscribe ("/velodyne_points", 1, cloud_callback);

    // Wait for transform between velodyne and stereo frames
    tf::TransformListener tf_listener;
    ROS_INFO("Waiting 10 seconds for tf between velodyne and stereo frames to become available...");
    // try{
    //   ros::Time now = ros::Time(0);
    //   if (tf_listener.waitForTransform(tf_frame_velodyne, tf_frame_stereo_l,
    //                  now, ros::Duration(10.0))){
    //     cout << "TF is available." << endl;
    //   }
    //   else cout << "Timed out waiting for TF." << endl;

    //   tf_listener.lookupTransform(tf_frame_velodyne, tf_frame_stereo_l, now, stereoL_tf_velodyne);
    //   // cout << "TF:" << stereoL_tf_velodyne << endl;
    // }
    // catch (const std::exception& e) {
    //   cerr << "Fatal exception while acquiring TF's: " << e.what();
    //   return -1;
    // }



    // Size adjustment ROI
    /*
      exFAST is currently heavily optimized for VGA size images, extracting a 
      properly sized region of interest is much more efficient than resizing the image
    */

    // Stereo matching parameters
    double uniqueness = 0.8;
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
    FeatureDetector* leftFeatureDetector = new ExtendedFAST(true, minThreshold, adaptivity, true, 10);
    FeatureDetector* rightFeatureDetector = new ExtendedFAST(false, minThreshold, adaptivity, true, 10);

    // Sparse Correspondences
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
        ros::spinOnce();  // still handle ROS callbacks if skipping an iteration
        loop_rate.sleep();
        cout << "Skipping this stereo pair for processing." << endl;
        continue;
      }
      try {
        // Containers
        Mat raw_left, raw_right, draw_left, draw_right;
        Mat_<unsigned char> leftImg, rightImg;

        // Thread Safety
        cout << "Getting frame data" << endl;
        left_mtx.lock();
        right_mtx.lock();
        {
          // Get Frame Message data
          namespace cvb = cv_bridge;
          cout << "left_most_recent: " << left_most_recent << endl;
          cout << "right_most_recent: " << right_most_recent << endl;
          input_bridge = cvb::toCvCopy(left_most_recent, sensor_msgs::image_encodings::BGR8);
          raw_left = input_bridge->image;
          draw_left = raw_left.clone();
          // imshow("raw_left", raw_left); waitKey(0);
          cout << "raw_left_size:"  << raw_left.size() << endl;
          input_bridge = cvb::toCvCopy(right_most_recent, sensor_msgs::image_encodings::BGR8);
          raw_right = input_bridge->image;
          draw_right = raw_right.clone();
        }
        cout << "Got frame data" << endl;
        left_mtx.unlock();
        right_mtx.unlock();

        // Check images for data and adjust size
        if(raw_left.data == NULL || raw_right.data == NULL)
          throw sparsestereo::Exception("Unable to open input images!");
        cvtColor(raw_left, leftImg, CV_BGR2GRAY);
        cvtColor(raw_right, rightImg, CV_BGR2GRAY);

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
        int rows = leftImg.rows;
        int cols = leftImg.cols;
        Mat_<char> charLeft(rows, cols);
        Mat_<char> charRight(rows, cols);
        Mat_<unsigned int> censusLeft(rows, cols);
        Mat_<unsigned int> censusRight(rows, cols);
        vector<KeyPoint> keypointsLeft, keypointsRight;

        ptime lastTime = microsec_clock::local_time(); // Time algorithm start time
        
        // Calculate census transforms for both images and detet features 
        // for both images in parallel
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
                
        // Stereo matching and reconstruction
        Point2f pt_L;
        Point2f pt_R;
        Matx31d pt_L_hom;
        Matx31d pt_R_hom;
        Mat X, X_hom;
        Vec3b color;

        // Calculate correspondences in Blue channel
        correspondences.clear();
        stereo.match(censusLeft, censusRight, keypointsLeft, keypointsRight, &correspondences);

        for(int i=0; i<(int)correspondences.size(); i++) {

          // Visualize feature points
          Scalar rand_color = Scalar(rand() % 256, rand() % 256, rand() % 256);
          rectangle(raw_left, pt_L - Point2f(2,2), pt_L + Point2f(2, 2), rand_color, CV_FILLED);
          rectangle(raw_right, pt_R - Point2f(2,2), pt_R + Point2f(2, 2), rand_color, CV_FILLED);

          // Triangulate matches to reconstruct 3D point
          pt_L = correspondences[i].imgLeft->pt;
          pt_R = correspondences[i].imgRight->pt;
          pt_L_hom = Matx31d(pt_L.x, pt_L.y, 1);
          pt_R_hom = Matx31d(pt_R.x, pt_R.y, 1);
          X_hom = nav::triangulate_Linear_LS(Mat(left_P), Mat(right_P), Mat(pt_L_hom), Mat(pt_R_hom));
          X_hom = X_hom / X_hom.at<double>(3, 0);

          // Get Color and fill point_cloud element precursor
          XYZRGB point;
          point.x = X_hom.at<double>(0, 0);
          point.y = X_hom.at<double>(1, 0);
          point.z = X_hom.at<double>(2, 0);
          Vec3b pix_color = raw_left.at<Vec3b>(pt_L);
          cout << i << "ptL: " << pt_L << " ptR: " << pt_R << " Color: " << pix_color 
            << " " << X_hom.t() << endl;
          point.b = pix_color[0];
          point.g = pix_color[1];
          point.r = pix_color[2];
          stereo_point_cloud.push_back(point);
        }

        // Display image 
        Mat screen = Mat::zeros(Size(leftImg.cols*2, leftImg.rows), CV_8UC3);
        raw_left.copyTo(screen(Rect(Point(0,0), leftImg.size())));
        raw_right.copyTo(screen(Rect(Point(leftImg.cols,0), leftImg.size())));
        namedWindow("Stereo");
        imshow("Stereo", screen);
        waitKey(0);

        // Print statistics
        time_duration elapsed = (microsec_clock::local_time() - lastTime);
        cout << "Time for stereo matching: " << elapsed.total_microseconds()/1.0e6 << "s" << endl
          << "Features detected in left image: " << keypointsLeft.size() << endl
          << "Features detected in right image: " << keypointsRight.size() << endl
          << "Percentage of matched features: " << (100.0 * correspondences.size() / keypointsLeft.size()) << "%" << endl;

        // Populate point cloud message header
        sensor_msgs::PointCloud2 pt_cloud_msg;
        header.seq = pt_cloud_seq++;
        ros::Time frame_time_l = left_most_recent_info->header.stamp;
        ros::Time frame_time_r = right_most_recent_info->header.stamp;
        ros::Duration offset((frame_time_r - frame_time_l).toSec() / 2);
        header.stamp = left_most_recent_info->header.stamp + offset;
        header.frame_id = tf_frame_stereo_l;
        pt_cloud_msg.header = header;
        cout << "pcd header: " <<pt_cloud_msg.header << endl;

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

        cout << "filling pcd2 msg " << endl;
        for(size_t i = 0; i < stereo_point_cloud.size(); ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_b, ++iter_g, ++iter_r)
        {
          *iter_x = stereo_point_cloud[i].x;
          *iter_y = stereo_point_cloud[i].y;
          *iter_z = stereo_point_cloud[i].z;
          *iter_b = stereo_point_cloud[i].b;
          *iter_g = stereo_point_cloud[i].g;
          *iter_r = stereo_point_cloud[i].r;
          cout << i << "XYZBGR: " << *iter_x << " " << *iter_y << " " << *iter_z << " " << int(*iter_b) << " " << int(*iter_g) << " " <<
           int(*iter_r) << endl;
         }

        // Publish point cloud
        pcl_pub.publish(pt_cloud_msg);

        // Empty point cloud container for upcoming iteration
        cout << "PCD size: " << pt_cloud_msg.height * pt_cloud_msg.row_step
          << ", " << stereo_point_cloud.size() << endl;
        stereo_point_cloud.clear();

        // Handle ROS callbacks
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
    std_srvs::SetBool::Request &req,
    std_srvs::SetBool::Response &resp) {
  stringstream ros_log;
  ros_log << "\x1b[1;31mSetting sereo point cloud generation to: \x1b[1;37m"
          << (req.data ? "on" : "off") << "\x1b[0m";
  ROS_INFO(ros_log.str().c_str());
  active = req.data;
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
    tf_frame_stereo_l = left_cam_model.tfFrame();
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
    right_P(0, 3) = -140;
    cout << "right projection_matrix: " << right_P << endl;
    tf_frame_stereo_r = right_cam_model.tfFrame();
    first_call = false;
  }
}

void cloud_callback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
     ROS_INFO("velodyne_points callback");
     sensor_msgs::PointCloud2 colored_velodyne_pcd;
     
}

