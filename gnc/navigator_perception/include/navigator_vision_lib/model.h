#pragma once
#include <string>
#include <vector>
#include <math.h>
#include <map>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

class PerceptionModel
{
public:

    PerceptionModel(float width, float height, int min_points);
    ~PerceptionModel();
    float width, height;
    int min_points;
    std::vector<Eigen::Vector3d> current_points;
     std::vector<cv::Point> current_points_2d;

    bool check_point(Eigen::Vector3d point, cv::Point point2d, cv::Mat img, cv::Matx34d left_cam_mat, bool check);
    void remove_point(Eigen::Vector3d point, cv::Point point2d);
    bool get_model(std::vector<Eigen::Vector3d>& model3d, std::vector<cv::Point>& model_position_2d, cv::Mat img, cv::Matx34d left_cam_mat);
    bool complete();
    void clear();


private:
    std::vector<std::vector<Eigen::Vector3d>> potential_models;
    std::vector<std::vector<cv::Point>> potential_models_2d;
    std::vector<float> unused_distances;
    std::map<std::string, float> point_to_distance;
    double cost_avg = 0;
    int count = 0;

    int get_furthest_point(std::vector<Eigen::Vector3d> my_model, int point);
    void visualize_points(std::vector<Eigen::Vector3d>  feature_pts_3d, cv:: Mat left_image, cv::Matx34d left_cam_mat, std::string name, double min_cost);
};
