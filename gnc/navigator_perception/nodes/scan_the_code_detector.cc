#include <navigator_vision_lib/scan_the_code_detector.hpp>

using namespace std;
using namespace cv;

///////////////////////////////////////////////////////////////////////////////////////////////////
// Class: StereoShapeDetector ////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

StereoShapeDetector::StereoShapeDetector()try :
  image_transport(nh),   rviz("/scan_the_code/visualization/detection")
  {
      stringstream log_msg;
      init_ros(log_msg);

      log_msg << "\nInitializing ScanTheCodeDetector:\n";
      int tab_sz = 4;

      object_tracker = new ObjectTracker();

      // Start main detector loop
      run_id = 0;

      boost::thread main_loop_thread(boost::bind(&StereoShapeDetector::run, this));
      main_loop_thread.detach();
      log_msg << setw(1 * tab_sz) << "" << "Running main detector loop in a background thread\n";

      log_msg << "ScanTheCodeDetector Initialized\n";
      ROS_INFO(log_msg.str().c_str());

  }
catch (const exception &e)
  {
      ROS_ERROR("Exception from within ScanTheCodeDetector constructor "
                "initializer list: ");
      ROS_ERROR(e.what());
  }



StereoShapeDetector::~StereoShapeDetector()
{
    ROS_INFO("Killed Stereo Shape Detector");
}

void StereoShapeDetector::validate_frame(Mat& current_image_left, Mat& current_image_right,
        Mat& processing_size_image_left, Mat& processing_size_image_right
                                           )
{
    // Prevent segfault if service is called before we get valid img_msg_ptr's
    if (left_most_recent.image_msg_ptr == NULL || right_most_recent.image_msg_ptr == NULL)
        {
            throw "ScanTheCodeDetector: Image Pointers are NULL.";

        }

    double sync_thresh = 0.5;
    // Get the most recent frames and camera info for both cameras
    cv_bridge::CvImagePtr input_bridge;
    try
        {

            left_mtx.lock();
            right_mtx.lock();

            // Left Camera
            input_bridge = cv_bridge::toCvCopy(left_most_recent.image_msg_ptr,
                                               sensor_msgs::image_encodings::BGR8);
            current_image_left = input_bridge->image;
            left_cam_model.fromCameraInfo(left_most_recent.info_msg_ptr);


           //std::cout<<current_image_left<<std::endl;

            resize(current_image_left, processing_size_image_left, Size(0, 0),
                   image_proc_scale, image_proc_scale);
            if (current_image_left.channels() != 3)
                {
                    throw "The left image topic does not contain a color image.";
                }

            // Right Camera
            input_bridge = cv_bridge::toCvCopy(right_most_recent.image_msg_ptr,
                                               sensor_msgs::image_encodings::BGR8);
            current_image_right = input_bridge->image;
            right_cam_model.fromCameraInfo(right_most_recent.info_msg_ptr);
            resize(current_image_right, processing_size_image_right, Size(0, 0),
                   image_proc_scale, image_proc_scale);
            if (current_image_right.channels() != 3)
                {
                    throw "The right image topic does not contain a color image.";
                }

            left_mtx.unlock();
            right_mtx.unlock();

        }
    catch (const exception &ex)
        {
            ROS_ERROR("[stereo_shape_detector] cv_bridge: Failed to convert images");
            left_mtx.unlock();
            right_mtx.unlock();
            throw "ROS ERROR";
        }

    // Enforce approximate image synchronization
    double left_stamp, right_stamp;
    left_stamp = left_most_recent.image_msg_ptr->header.stamp.toSec();
    right_stamp = right_most_recent.image_msg_ptr->header.stamp.toSec();
    double sync_error = fabs(left_stamp - right_stamp);
    stringstream sync_msg;
    sync_msg << "Left and right images were not sufficiently synchronized"
             << "\nsync error: " << sync_error << "s";
    if (sync_error > sync_thresh)
        {
            ROS_WARN(sync_msg.str().c_str());
            throw "Sync Error";
        }
}

void StereoShapeDetector::init_ros(stringstream& log_msg)
{
    using ros::param::param;
    int tab_sz = 4;
    // Default parameters
    string img_topic_left_default = "/stereo/left/image_rect_color/";
    string img_topic_right_default = "/stereo/right/image_rect_color/";
    string activation_default = "/stereo_shape_detector/activation_switch";
    float image_proc_scale_default = 0.5;
    int diffusion_time_default = 1;
    int max_features_default = 35;
    int feature_block_size_default = 11;
    float feature_min_distance_default = 5.0;


    // Set image processing scale
    image_proc_scale = param<float>("/stereo_shape_detector/img_proc_scale", image_proc_scale_default);
    log_msg << setw(1 * tab_sz) << "" << "Image Processing Scale: \x1b[37m"
            << image_proc_scale << "\x1b[0m\n";

    // Set diffusion duration in pseudotime
    diffusion_time = param<int>("/stereo_shape_detector/diffusion_time", diffusion_time_default);
    log_msg << setw(1 * tab_sz) << "" << "Anisotropic Diffusion Duration: \x1b[37m"
            << diffusion_time << "\x1b[0m\n";

    // Set feature extraction parameters
    max_features = param<int>("/stereo_shape_detector/max_features", max_features_default);
    log_msg << setw(1 * tab_sz) << "" << "Maximum features: \x1b[37m"
            << max_features << "\x1b[0m\n";
    feature_block_size = param<int>("/stereo_shape_detector/feature_block_size",
                                    feature_block_size_default);
    log_msg << setw(1 * tab_sz) << "" << "Feature Block Size: \x1b[37m"
            << feature_block_size << "\x1b[0m\n";
    feature_min_distance = param<float>("/stereo_shape_detector/feature_min_distance",
                                        feature_min_distance_default);
    log_msg << setw(1 * tab_sz) << "" << "Feature Minimum Distance: \x1b[37m"
            << feature_min_distance << "\x1b[0m\n";

    // Subscribe to Cameras (image + camera_info)
    string left = param<string>("/stereo_shape_detector/input_left", img_topic_left_default);
    string right = param<string>("/stereo_shape_detector/input_right", img_topic_right_default);
    left_image_sub = image_transport.subscribeCamera(
                         left, 10, &StereoShapeDetector::left_image_callback, this);
    right_image_sub = image_transport.subscribeCamera(
                          right, 10, &StereoShapeDetector::right_image_callback, this);
    log_msg << setw(1 * tab_sz) << "" << "Camera Subscriptions:\x1b[37m\n"
            << setw(2 * tab_sz) << "" << "left  = " << left << endl
            << setw(2 * tab_sz) << "" << "right = " << right << "\x1b[0m\n";


    active = false;
    string activation = param<string>("/stereo_shape_detector/activation", activation_default);

    detection_switch = nh.advertiseService(activation, &StereoShapeDetector::detection_activation_switch, this);

    log_msg
            << setw(1 * tab_sz) << "" << "Advertised stereo shape detector board detection switch:\n"
            << setw(2 * tab_sz) << "" << "\x1b[37m" << activation << "\x1b[0m\n";
}


void StereoShapeDetector::run()
{
    ros::Rate loop_rate(10);  // process images 10 times per second

    while (ros::ok())
        {
            if (active)
            {
                process_current_images();
            }
            loop_rate.sleep();
        }
    return;
}

void StereoShapeDetector::process_current_images()
{
    Mat current_image_left, current_image_right, processing_size_image_left,
        processing_size_image_right;
    try
    {
        validate_frame(current_image_left,
                          current_image_right,
                          processing_size_image_left,
                          processing_size_image_right);
    }
    catch(const char* msg)
    {
        ROS_ERROR(msg);
        return;
    }

    if(mission_complete){
      return;
    }

    if(looking_for_model)
    {
      Matx34d left_cam_mat = left_cam_model.fullProjectionMatrix();
      Matx34d  right_cam_mat = right_cam_model.fullProjectionMatrix();
      vector<Eigen::Vector3d> position;
      vector<cv::Point> position2d;

      bool got_model = model_fitter->determine_model_position(position,
                                             position2d,
                                             max_features,
                                             feature_block_size,
                                             feature_min_distance,
                                             image_proc_scale,
                                             diffusion_time,
                                             current_image_left,
                                             current_image_right,
                                             left_cam_mat,
                                             right_cam_mat
                                             );


      if(got_model){
        cv::Mat l_diffused,r_diffused;
        model_fitter->denoise_images(l_diffused, r_diffused,
                                               diffusion_time, current_image_left,
                                               current_image_right);

        object_tracker->begin_tracking_object(position2d, l_diffused);
        looking_for_model = false;
        tracking_model = true;
        color_tracker->set_status(true);
      }

    }else if(tracking_model){
      std::vector<cv::Point2f> corners;
      cv::Mat l_diffused,r_diffused;
      model_fitter->denoise_images(l_diffused, r_diffused,
                                             diffusion_time, current_image_left,
                                             current_image_right);


      bool found_object = object_tracker->track_object(l_diffused, corners);
      if(found_object){
          mission_complete = color_tracker->track(current_image_left, corners, image_proc_scale);
      }else{
        looking_for_model = true;
        tracking_model = false;
        object_tracker->clear();
        color_tracker->clear();
        color_tracker->set_status(false);
      }
    }

    // IMBN: make state machiney type thing

}

bool StereoShapeDetector::detection_activation_switch(
    navigator_msgs::StereoShapeDetector::Request &req,
    navigator_msgs::StereoShapeDetector::Response &resp)
{
    stringstream ros_log;
    ROS_INFO(ros_log.str().c_str());
    mission_complete = false;

    if(!req.detection_switch){
      ros_log << "Shape Detector switch turned off" << req.detection_switch <<"\n";
      ROS_INFO(ros_log.str().c_str());
      resp.success = true;
      return true;
    }

    ros_log << "Shape Detector switch turned on" << req.detection_switch <<"\n";
    ros_log<<  "Detecting shape "<< req.shape <<"\n";
    ros_log << "Shape Detector shape: " << req.shape <<"\n";
    ros_log << "Shape Detector processing type: " << req.processing_type <<"\n";
    ros_log << "Shape Detector num_points: " << req.num_points <<"\n";
    ROS_INFO(ros_log.str().c_str());

    std::vector<float> params;
    for(float f : req.model_params){
      params.push_back(f);
    }

    // This seems unnecessary now, but I am adding on to this later,
    // to make this code more general to any shape.
    if(req.shape == "RectangleModel"){

      PerceptionModel model = PerceptionModel(params,req.num_points);
      this->model_fitter = new StereoModelFitter(model);

    }
    if(req.processing_type == "ColorTracker"){
      color_tracker = new ColorTracker();
    }
    active = req.detection_switch;
    resp.success = true;
    return true;
}

void StereoShapeDetector::left_image_callback(
    const sensor_msgs::ImageConstPtr &image_msg_ptr,
    const sensor_msgs::CameraInfoConstPtr &info_msg_ptr)
{
    left_mtx.lock();
    left_most_recent.image_msg_ptr = image_msg_ptr;
    left_most_recent.info_msg_ptr = info_msg_ptr;
    left_mtx.unlock();
}

void StereoShapeDetector::right_image_callback(
    const sensor_msgs::ImageConstPtr &image_msg_ptr,
    const sensor_msgs::CameraInfoConstPtr &info_msg_ptr)
{
    right_mtx.lock();
    right_most_recent.image_msg_ptr = image_msg_ptr;
    right_most_recent.info_msg_ptr = info_msg_ptr;
    right_mtx.unlock();
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "stereo_shape_detector");
    ROS_INFO("Initializing node /stereo_shape_detector");
    StereoShapeDetector stereo_shape_detector;
    ros::spin();

}
