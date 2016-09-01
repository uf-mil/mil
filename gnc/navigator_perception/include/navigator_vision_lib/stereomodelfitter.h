#pragma once
#include <string>
#include <vector>
#include <utility>
#include <iostream>
#include <algorithm>
#include <stdexcept>
#include <cstdint>
#include <iterator>

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

class StereoModelFitter
{
public:

    StereoModelFitter(PerceptionModel model, image_transport::Publisher debug_publisher);
    PerceptionModel model;

    bool determine_model_position(Eigen::Vector3d& position, int max_corners, int block_size, double min_distance, double image_proc_scale, int diffusion_time,
                                  Mat current_image_left, Mat current_image_right,
                                  Matx34d left_cam_mat, Matx34d right_cam_mat);

protected:
    void visualize_points(std::vector<Eigen::Vector3d>  feature_pts_3d, Mat& current_image_left);
    bool check_for_model(vector<Eigen::Vector3d>  feature_pts_3d, vector<Eigen::Vector3d>& correct_model, vector<cv::Point> correct_image_points);
    void calculate_3D_reconstruction(std::vector<Eigen::Vector3d>& feature_pts_3d, vector<int> correspondence_pair_idxs, vector<Point> features_l,
                                     vector<Point> features_r);
    void get_corresponding_pairs(std::vector<int>& correspondence_pair_idxs,  vector<Point> features_l,
                                 vector<Point> features_r, int picture_width);
    void extract_features(std::vector<Point> & features, Mat& image, int max_corners, int block_size, double quality_level, double min_distance);
    void denoise_images(Mat& l_diffused, Mat& r_diffused, int diffusion_time, Mat current_image_left,
                        Mat current_image_right);
    void decision_tree(vector<Eigen::Vector3d>  feature_pts_3d, int curr, int left, vector<int> check);
    std::vector<int> split(string str);

    cv::Mat* current_image_left = NULL;
    cv::Mat* current_image_right = NULL;
    cv::Matx34d* left_cam_mat = NULL;
    cv::Matx34d* right_cam_mat = NULL;


private:
    double image_proc_scale;
    image_transport::Publisher debug_publisher;
};


// Combinations of k elements from a set of size n (indexes)
void combinations(uint8_t n, uint8_t k, std::vector< std::vector<uint8_t> > &idx_array);

// Edge preserving image denoising
void anisotropic_diffusion(const cv::Mat &src, cv::Mat &dest, int t_max);

void _increase_elements_after_level(vector<uint8_t> comb, vector< vector<uint8_t> > &comb_array,
                                    uint8_t n, uint8_t k, uint8_t level);
