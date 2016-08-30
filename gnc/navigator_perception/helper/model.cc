#include <navigator_vision_lib/model.h>


PerceptionModel::PerceptionModel(float width, float height, int min_points): width(width), height(height), min_points(min_points)
{
    unused_distances.push_back(width);
    unused_distances.push_back(height);
    unused_distances.push_back(sqrt(pow(width, 2) + pow(height, 2)));
}

PerceptionModel::~PerceptionModel()
{
}

bool PerceptionModel::check_point(Eigen::Vector3d point)
{
    current_points.push_back(point);
    if(current_points.size()  == 1)
        {
            return true;
        }

    Eigen::Vector3d starting_point = current_points[0];
    Eigen::Vector3d diff = starting_point - point;
    float dist_from_starting = sqrt(pow(diff[0], 2) + pow(diff[1], 2) + pow(diff[2], 2));
    float err = ((width + height) / 2) * .05;
    for(float dist : unused_distances)
        {
            float upper = dist + err;
            float lower = dist - err;
            if(dist_from_starting > lower && dist_from_starting < upper)
                {
                    unused_distances.erase(std::remove(unused_distances.begin(), unused_distances.end(), dist), unused_distances.end());
                    point_to_distance.insert(std::pair<Eigen::Vector3d*,float>(&point, dist));
                    if(current_points.size()  == min_points)
                        {
                            // This is supposed to copy current_poitns into a new element of potential points.
                            potential_models.push_back(current_points);
                        }
                    return true;
                }
        }
    return false;
}

void PerceptionModel::remove_point(Eigen::Vector3d point)
{
    current_points.erase(std::remove(current_points.begin(), current_points.end(), point), current_points.end());
    float dist = point_to_distance[&point];
    unused_distances.push_back(dist);
    point_to_distance.erase(&point);
}

bool PerceptionModel::get_model(std::vector<Eigen::Vector3d>& model3d, std::vector<Eigen::Vector3d>& model2d)
{

}

bool PerceptionModel::complete()
{
    if(current_points.size()  == 4)
        {
            return true;
        }
    return true;
}
