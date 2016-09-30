#include <navigator_vision_lib/model.h>


PerceptionModel::PerceptionModel(float width, float height, int min_points)
                                : width(width),
                                  height(height),
                                  min_points(min_points)
{
    unused_distances.push_back(width);
    unused_distances.push_back(height);
    unused_distances.push_back(sqrt(pow(width, 2) + pow(height, 2)));
}

PerceptionModel::~PerceptionModel()
{
}


void PerceptionModel::clear(){
  potential_models.clear();
}

bool PerceptionModel::check_point(Eigen::Vector3d point, cv:: Mat img, cv::Matx34d left_cam_mat, bool check)

{
    current_points.push_back(point);
    // If this is the first point of the 4 it automatically passes
    if(current_points.size()  == 1)
    {
        return true;
    }
    Eigen::Vector3d starting_point = current_points[0];
    float dist_from_starting = (starting_point - point).norm();
    bool point_fits_model = false;
    double mindist = 100;
    double minel = -1;
    // Go through all the distances that haven't been used yet for this model
    for(float dist : unused_distances)
    {
        float err = dist * .4;
        float upper = dist + err;
        float lower = dist - err;

        //if the point is within some range of the current distance, then it's a point that fits the current model
        if(dist_from_starting > lower && dist_from_starting < upper)
        {

            // update the min
            double m = fabs(dist_from_starting);
            if(m < mindist){
              minel = dist;
              mindist = m;
            }

            std::stringstream ss;
            ss << point(0) << point(1) <<point(2);
            // maps the point to the distance, so later it can be added back to unused distances
            point_to_distance.insert(std::pair<std::string,float>(ss.str(), dist));
            // If this is the final point in our model, add the full model to the list of potential models
            if(current_points.size()  == min_points)
            {
                potential_models.push_back(current_points);
            }
            point_fits_model = true;
        }
    }

    // If the point fit the model, remove its corresponding distance from list of unused distances
    if(point_fits_model){
       unused_distances.erase(std::remove(unused_distances.begin(), unused_distances.end(), minel), unused_distances.end());
    }


    return point_fits_model;
}

void PerceptionModel::remove_point(Eigen::Vector3d point)
{
    current_points.erase(std::remove(current_points.begin(), current_points.end(), point), current_points.end());
    std::stringstream ss;
    ss << point(0) << point(1) << point(2);
    if(point_to_distance.find( ss.str()) != point_to_distance.end()){
      float dist = point_to_distance[ss.str()];
      unused_distances.push_back(dist);
      point_to_distance.erase(ss.str());
    }
}

void PerceptionModel::visualize_points(std::vector<Eigen::Vector3d>  feature_pts_3d, cv:: Mat img, cv::Matx34d left_cam_mat, std::string name)
{
    cv::Mat current_image_left = img.clone();
    // visualize reconstructions
    for(size_t i = 0; i < feature_pts_3d.size(); i++)
        {
            Eigen::Vector3d pt = feature_pts_3d[i];
            cv::Matx41d position_hom(pt(0), pt(1), pt(2), 1);
            cv::Matx31d pt_L_2d_hom = left_cam_mat * position_hom;
            cv::Point2d L_center2d(pt_L_2d_hom(0) / pt_L_2d_hom(2), pt_L_2d_hom(1) / pt_L_2d_hom(2));
            cv::Scalar color(255, 0, 255);
            std::stringstream label;
            label << i;
            cv::circle(current_image_left, L_center2d, 5, color, -1);
            cv::putText(current_image_left, label.str(), L_center2d, cv::FONT_HERSHEY_SIMPLEX,
                    0.0015 * current_image_left.rows, cv::Scalar(0, 0, 0), 2);
        }

    cv::imshow(name, current_image_left);
    cv::waitKey(33);
}


bool PerceptionModel::get_model(std::vector<Eigen::Vector3d>& model3d, cv::Mat left_image, cv::Matx34d left_cam_mat)
{
    double min_cost = 100000;
    std::vector<Eigen::Vector3d> min_model;

    // Go through every potential model
    for(std::vector<Eigen::Vector3d> mymodel: potential_models){
        double cost = 0;
        // Go through every point in this model
        for(int i = 0; i != min_points; ++i){
          Eigen::Vector3d a = mymodel[i];
          Eigen::Vector3d b;
          Eigen::Vector3d c;
          bool b_init = true;
          // Get the furtherst point away from the current point
          int furthest_point = get_furthest_point(mymodel, i);
          // Select the other to points, other than the current and the furthest away
          for(int j = 0; j != min_points; ++j){
            if(j != furthest_point && j != i && b_init){
               b_init = false;
               b = mymodel[j];
            }else if(j != furthest_point && j != i){
               c = mymodel[j];
               break;
            }
          }
          // Get the angles between the vectors
          Eigen::Vector3d e = b - a;
          Eigen::Vector3d f = c - a;
          e.normalize();
          f.normalize();
          double h = e.dot(f);
          double t1 = acos(h);
          // The farther away from 90 degrees the points are the larger the cost of this model
          double x = (10 * fabs(t1 - M_PI/2));
          cost += x;

        }

        if(cost < min_cost){
          min_cost = cost;
          min_model = mymodel;
        }
    }
    model3d = min_model;
    if(potential_models.size() != 0 && min_cost < 4){
      //visualize_points(min_model, left_image, left_cam_mat, "WINNER");
    }
    return true;
}

bool PerceptionModel::complete()
{
    if(current_points.size()  == 4)
    {
        return true;
    }
    return false;
}

int PerceptionModel::get_furthest_point(std::vector<Eigen::Vector3d> mymodel, int point){
  float max_val = 0;
  int max = -1;
  for(int j = 0; j != 4; ++j){
    if(j == point) continue;
    float dist_from_starting = (mymodel[point] - mymodel[j]).norm();
    if(dist_from_starting > max_val){
      max_val = dist_from_starting;
      max = j;
    }
  }
  return max;
}
