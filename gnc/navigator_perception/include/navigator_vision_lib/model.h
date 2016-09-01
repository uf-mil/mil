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
    bool check_point(Eigen::Vector3d point, cv::Mat img, cv::Matx34d left_cam_mat, bool check);
    void remove_point(Eigen::Vector3d point);
    bool get_model(std::vector<Eigen::Vector3d>& model3d, std::vector<cv::Point>& model2d, cv::Mat img, cv::Matx34d left_cam_mat);
    bool complete();
    void clear();
    std::vector<Eigen::Vector3d> current_points;

private:
    std::vector<std::vector<Eigen::Vector3d>> potential_models;

    std::vector<float> unused_distances;
    std::map<std::string, float> point_to_distance;

    int which_point( Eigen::Vector3d a,  Eigen::Vector3d b);



    void visualize_points(std::vector<Eigen::Vector3d>  feature_pts_3d, cv:: Mat img, cv::Matx34d left_cam_mat, std::string name);
};
