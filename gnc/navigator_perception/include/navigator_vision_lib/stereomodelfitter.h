#pragma once
#include <string>
#include <vector>
#include <utility>
#include <iostream>
#include <algorithm>
#include <stdexcept>
#include <cstdint>
#include <iterator>

#include <math.h>
#include <stdio.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/StdVector>
#include <chrono>
#include <thread>

#include <ros/ros.h>
#include <boost/algorithm/string.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/image_encodings.h>


#include <navigator_vision_lib/cv_tools.hpp>
#include <navigator_vision_lib/model.h>

using namespace cv;
using namespace std;

class StereoModelFitter
{
public:

    StereoModelFitter(PerceptionModel model);
    PerceptionModel model;

    bool determine_model_position(vector<Eigen::Vector3d>& model_position,
                                  vector<cv::Point>& model_position_2d,
                                  int max_corners,
                                  int block_size,
                                  double min_distance,
                                  double image_proc_scale,
                                  int diffusion_time,
                                  Mat current_image_left,
                                  Mat current_image_right,
                                  Matx34d left_cam_mat,
                                  Matx34d right_cam_mat);


    void denoise_images(Mat& l_diffused,
                        Mat& r_diffused,
                        int diffusion_time,
                        Mat current_image_left,
                        Mat current_image_right);

protected:
    bool check_for_model(vector<Eigen::Vector3d>  feature_pts_3d,
                         vector<cv::Point> left_points_2d,
                         vector<Eigen::Vector3d>& correct_model,
                         vector<cv::Point>& model_position_2d);

    void calculate_3D_reconstruction(std::vector<Eigen::Vector3d>& feature_pts_3d,
                                     vector<Point> features_l,
                                     vector<Point> features_r);

    void get_corresponding_pairs(cv::Mat frame_l,
                                 cv::Mat frame_r,
                                 vector<Point> features_l,
                                 vector<Point> features_r,
                                 vector<Point>& features_l_out,
                                 vector<Point>& features_r_out,
                                 int picture_width);

    void extract_features(std::vector<Point> & features,
                          Mat& image,
                          int max_corners,
                          int block_size,
                          double quality_level,
                          double min_distance);


    void decision_tree(vector<Eigen::Vector3d> feature_pts_3d,
                       vector<cv::Point> left_points_2d,
                       int curr,
                       int remaining,
                       vector<int> debug_points,
                       bool debug);

    void visualize_points(std::vector<Eigen::Vector3d> feature_pts_3d,
                          Mat& current_image_left);


    cv::Mat* current_image_left = NULL;
    cv::Mat* current_image_right = NULL;
    cv::Matx34d* left_cam_mat = NULL;
    cv::Matx34d* right_cam_mat = NULL;


private:
    double image_proc_scale;
    ros::NodeHandle nh;
    image_transport::ImageTransport image_transport = image_transport::ImageTransport(nh);
    image_transport::Publisher debug_image_3dpoints = image_transport.advertise("stereo_model_fitter/debug_img/3dpoints", 1, true);
    image_transport::Publisher debug_image_2dpoints = image_transport.advertise("stereo_model_fitter/debug_img/2dpoints", 1, true);


};




// Edge preserving image denoising
void anisotropic_diffusion(const cv::Mat &src, cv::Mat &dest, int t_max);

std::vector<int> split(string str);

