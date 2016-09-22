#include <navigator_vision_lib/scan_the_code_detector.hpp>

using namespace std;
using namespace cv;

///////////////////////////////////////////////////////////////////////////////////////////////////
// Class: ScanTheCodeDetector ////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

ScanTheCodeDetector::ScanTheCodeDetector()
try :
    image_transport(nh),   rviz("/scan_the_code/visualization/detection")
    {

        stringstream log_msg;
        init_ros(log_msg);

        log_msg << "\nInitializing ScanTheCodeDetector:\n";
        int tab_sz = 4;

        PerceptionModel model = PerceptionModel(.1905,.381,4);
        model_fitter = new StereoModelFitter(model, debug_image_pub);
        // Start main detector loop
        run_id = 0;

        boost::thread main_loop_thread(boost::bind(&ScanTheCodeDetector::run, this));
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

ScanTheCodeDetector::~ScanTheCodeDetector()
{
    ROS_INFO("Killed Torpedo Board Detector");
}

void ScanTheCodeDetector::validate_frame(Mat& current_image_left, Mat& current_image_right,
        Mat& processing_size_image_left, Mat& processing_size_image_right
                                           )
{
    // Prevent segfault if service is called before we get valid img_msg_ptr's
    if (left_most_recent.image_msg_ptr == NULL || right_most_recent.image_msg_ptr == NULL)
        {
            throw "Torpedo Board Detector: Image Pointers are NULL.";

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
            ROS_ERROR("[scan_the_code] cv_bridge: Failed to convert images");
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

void ScanTheCodeDetector::init_ros(stringstream& log_msg)
{
    using ros::param::param;
    int tab_sz = 4;
    // Default parameters
    string img_topic_left_default = "/stereo/left/image_rect_color/";
    string img_topic_right_default = "/stereo/right/image_rect_color/";
    string activation_default = "/scan_the_code/detection_activation_switch";
    string dbg_topic_default = "/scan_the_code/dbg_imgs";
    string pose_est_srv_default = "/scan_the_code/pose_est_srv";
    float image_proc_scale_default = 0.5;
    int diffusion_time_default = 5;
    int max_features_default = 35;
    int feature_block_size_default = 11;
    float feature_min_distance_default = 15.0;
    bool generate_dbg_img_default = true;
    int frame_height_default = 644;
    int frame_width_default = 482;



    // Set image processing scale
    image_proc_scale = param<float>("/scan_the_code_vision/img_proc_scale", image_proc_scale_default);
    log_msg << setw(1 * tab_sz) << "" << "Image Processing Scale: \x1b[37m"
            << image_proc_scale << "\x1b[0m\n";

    // Set diffusion duration in pseudotime
    diffusion_time = param<int>("/scan_the_code_vision/diffusion_time", diffusion_time_default);
    log_msg << setw(1 * tab_sz) << "" << "Anisotropic Diffusion Duration: \x1b[37m"
            << diffusion_time << "\x1b[0m\n";

    // Set feature extraction parameters
    max_features = param<int>("/scan_the_code_vision/max_features", max_features_default);
    log_msg << setw(1 * tab_sz) << "" << "Maximum features: \x1b[37m"
            << max_features << "\x1b[0m\n";
    feature_block_size = param<int>("/scan_the_code_vision/feature_block_size",
                                    feature_block_size_default);
    log_msg << setw(1 * tab_sz) << "" << "Feature Block Size: \x1b[37m"
            << feature_block_size << "\x1b[0m\n";
    feature_min_distance = param<float>("/scan_the_code_vision/feature_min_distance",
                                        feature_min_distance_default);
    log_msg << setw(1 * tab_sz) << "" << "Feature Minimum Distance: \x1b[37m"
            << feature_min_distance << "\x1b[0m\n";

    // Configure debug image generation
    generate_dbg_img = param<bool>("/scan_the_code_vision/generate_dbg_imgs", generate_dbg_img_default);

    // Subscribe to Cameras (image + camera_info)
    string left = param<string>("/scan_the_code_vision/input_left", img_topic_left_default);
    string right = param<string>("/scan_the_code_vision/input_right", img_topic_right_default);
    left_image_sub = image_transport.subscribeCamera(
                         left, 10, &ScanTheCodeDetector::left_image_callback, this);
    right_image_sub = image_transport.subscribeCamera(
                          right, 10, &ScanTheCodeDetector::right_image_callback, this);
    log_msg << setw(1 * tab_sz) << "" << "Camera Subscriptions:\x1b[37m\n"
            << setw(2 * tab_sz) << "" << "left  = " << left << endl
            << setw(2 * tab_sz) << "" << "right = " << right << "\x1b[0m\n";

    // Register Pose Estimation Service Client
    string pose_est_srv = param<string>("/scan_the_code_vision/pose_est_srv", pose_est_srv_default);
    pose_client = nh.serviceClient<sub8_msgs::TorpBoardPoseRequest>( pose_est_srv);
    log_msg
            << setw(1 * tab_sz) << "" << "Registered as client of the service:\n"
            << setw(2 * tab_sz) << "" << "\x1b[37m" << pose_est_srv << "\x1b[0m\n";

    // Advertise debug image topic
    string dbg_topic = param<string>("/scan_the_code_vision/dbg_imgs", dbg_topic_default);
    debug_image_pub = image_transport.advertise(dbg_topic, 1, true);
    log_msg << setw(1 * tab_sz) << "" << "Advertised debug image topic:\n"
            << setw(2 * tab_sz) << "" << "\x1b[37m" << dbg_topic << "\x1b[0m\n";

    // Setup debug image quadrants
    int frame_height = param<int>("/scan_the_code_vision/frame_height", frame_height_default);
    int frame_width = param<int>("/scan_the_code_vision/frame_width", frame_width_default);
    Size input_frame_size(frame_height, frame_width);  // This needs to be changed if we ever change
    // the camera settings for frame size
    Size proc_size(cvRound(image_proc_scale * input_frame_size.width),
                   cvRound(image_proc_scale * input_frame_size.height));
    Size dbg_img_size(proc_size.width * 2, proc_size.height * 2);
    debug_image = Mat(dbg_img_size, CV_8UC3, Scalar(0, 0, 0));
    upper_left = Rect(Point(0, 0), proc_size);
    upper_right = Rect(Point(proc_size.width, 0), proc_size);
    lower_left = Rect(Point(0, proc_size.height), proc_size);
    lower_right = Rect(Point(proc_size.width, proc_size.height), proc_size);
    // Advertise detection activation switch
    active = false;
    string activation = param<string>("/scan_the_code_vision/activation", activation_default);
    detection_switch = nh.advertiseService(
                           activation, &ScanTheCodeDetector::detection_activation_switch, this);
    log_msg
            << setw(1 * tab_sz) << "" << "Advertised scan_the_code board detection switch:\n"
            << setw(2 * tab_sz) << "" << "\x1b[37m" << activation << "\x1b[0m\n";
}


void ScanTheCodeDetector::run()
{
    ros::Rate loop_rate(10);  // process images 10 times per second
    while (ros::ok())
        {
            if (true)
                {
                    process_current_images();

                }
            loop_rate.sleep();
        }

    return;
}

void ScanTheCodeDetector::process_current_images()
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

    Matx34d left_cam_mat = left_cam_model.fullProjectionMatrix();
    Matx34d  right_cam_mat = right_cam_model.fullProjectionMatrix();
    vector<Eigen::Vector3d> position;
    model_fitter->determine_model_position(position,
                                           max_features,
                                           feature_block_size,
                                           feature_min_distance,
                                           image_proc_scale,
                                           diffusion_time,
                                           current_image_left,
                                           current_image_right,
                                           left_cam_mat,
                                           right_cam_mat);
}

bool ScanTheCodeDetector::detection_activation_switch(
    sub8_msgs::TBDetectionSwitch::Request &req,
    sub8_msgs::TBDetectionSwitch::Response &resp)
{
    resp.success = false;
    stringstream ros_log;
    ros_log << "\x1b[1;31mSetting torpedo board detection to: \x1b[1;37m"
            << (req.detection_switch ? "on" : "off") << "\x1b[0m";
    ROS_INFO(ros_log.str().c_str());
    active = req.detection_switch;
    if (active == req.detection_switch)
        {
            resp.success = true;
            return true;
        }
    return false;
}

void ScanTheCodeDetector::left_image_callback(
    const sensor_msgs::ImageConstPtr &image_msg_ptr,
    const sensor_msgs::CameraInfoConstPtr &info_msg_ptr)
{
    left_mtx.lock();
    left_most_recent.image_msg_ptr = image_msg_ptr;
    left_most_recent.info_msg_ptr = info_msg_ptr;
    left_mtx.unlock();
}

void ScanTheCodeDetector::right_image_callback(
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
    ros::init(argc, argv, "scan_the_code_perception");
    ROS_INFO("Initializing node /scan_the_code_perception");
    ScanTheCodeDetector scan_the_code_detector;
    ros::spin();
}
