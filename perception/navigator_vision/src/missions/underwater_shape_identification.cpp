#include <missions/underwater_shape_identification.hpp>

namespace nav {

using namespace std;
using namespace cv;

namespace fs = boost::filesystem;

vector<Shape> Shape::loadShapes(string directory, float shape_area)
{
  fs::directory_iterator dir_end;  // default ctor --> past-the-end
  fs::path dir{directory};
  vector<string> img_filenames;
  if(fs::is_directory(dir)){

    // iterate through all files in directory
    for(fs:: directory_iterator dir_itr(dir); dir_itr != dir_end; ++dir_itr){
      // collect all files that match img extension
      string path_str {dir_itr->path().filename().string()};
      if(path_str.compare(path_str.size() - 3, 3, "png") == 0){
        img_filenames.push_back(dir_itr->path().string());
      }
    }
  }
  else {
    cerr << "Error: " << dir << " is not a valid directory." << endl;
  }

  vector<Shape> shapes;
  for(auto path : img_filenames)
  {
    shapes.push_back(Shape());
    shapes.back().load(path, shape_area);
    if(!shapes.back().ok())
      shapes.pop_back();
  }

  return shapes;
}

void Shape::load(string path, float shape_area)
{
  _ok = true;
  fs::path template_path{path};
  if(!fs::exists(template_path))
  {
    cout << __PRETTY_FUNCTION__ << ": the file " << path << "doesn't exist." << endl;
    _ok = false;
    return;
  }
 
  _name = template_path.stem().string();
  _template_path =  path;
  _template = cv::imread(path, CV_LOAD_IMAGE_GRAYSCALE);
  if(!_template.data)
  {
    cout << __PRETTY_FUNCTION__ << ": the image at " << path << " could not be loaded." << endl;
    _ok = false;
    return;
  }

  float template_area = _template.rows * _template.cols;
  if(shape_area == 0.0f)
  {
    cout << __PRETTY_FUNCTION__ << ": shape_area can't be zero." << endl;
    _ok = false;
    return;
  }
  _pixels_per_meter = sqrt(template_area / shape_area);

  // Find angle of radial symmetry
  _radial_symmetry_angle = getRadialSymmetryAngle(_template, 0.01);
}

UnderwaterShapeDetector::UnderwaterShapeDetector(ros::NodeHandle &nh, int img_buf_size, string name_space)
: _nh(nh), _camera(_nh, img_buf_size), _ns(name_space)
{
  string camera_topic;
  _nh.param<string>(_ns + "template_directory", _template_dir, "templates/underwater_shape_identification/");
  _nh.param<float>(_ns + "shape_area", _shape_area, 1.0f);
  _nh.param<string>(_ns + "camera_topic", camera_topic, "down/image_rect");
  _nh.param<float>(_ns + "search_depth", _search_depth, 5.0f);
  _nh.param<float>(_ns + "depth_uncertainty", _depth_uncertainty, 1.0f);
  _nh.param<float>(_ns + "min_boat_displacement", _min_boat_displacement, 0.1);
  _nh.param<int>(_ns + "min_detections", _min_detections, 10);
  _nh.param<map<string, bool>>(_ns + "is_target", _is_target,
                               {{"circle", true}, {"cross", true}, {"triangle", true}});
  _nh.param<map<string, float>>(_ns + "detection_thresholds", _detection_thresholds,
                               {{"circle", 0.5}, {"cross", 0.5}, {"triangle", 0.5}});

  string pkg_path = ros::package::getPath("navigator_vision");
  auto shapes = Shape::loadShapes(pkg_path + "/" + _template_dir, _shape_area);

  // Eliminate shapes without thresholds or not targets
  for(auto it = shapes.end() - 1; it >= shapes.begin(); it--)
  {
    if(_detection_thresholds.count(it->name()) < 1 || !_is_target.at(it->name()))
      shapes.erase(it);
  }

  // Add remaining shapes to map
  for(auto& sh : shapes)
    _shapes[sh.name()] = sh;
}

}  // namespace nav

