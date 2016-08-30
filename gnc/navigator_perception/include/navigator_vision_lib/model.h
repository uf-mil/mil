#pragma once
#include <string>
#include <vector>
#include <math.h>
#include <map>
#include <Eigen/Core>

class PerceptionModel
{
public:

    PerceptionModel(float width, float height, int min_points);
    ~PerceptionModel();
    float width, height;
    int min_points;
    bool check_point(Eigen::Vector3d point);
    void remove_point(Eigen::Vector3d point);
    bool get_model(std::vector<Eigen::Vector3d>& model3d, std::vector<Eigen::Vector3d>& model2d);
    bool complete();

private:
    std::vector<std::vector<Eigen::Vector3d>> potential_models;
    std::vector<Eigen::Vector3d> current_points;
    std::vector<float> unused_distances;
    std::map<Eigen::Vector3d *, float> point_to_distance;
};
