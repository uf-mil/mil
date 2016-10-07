

#include "ShapeDetector.h"
float ShapeDetector::CROSS_BOUNDING_AREA_LOW = 0.43;
float ShapeDetector::CROSS_BOUNDING_AREA_HIGH = 0.47;
float ShapeDetector::TRI_BOUNDING_AREA_LOW = 0.41;
float ShapeDetector::TRI_BOUNDING_AREA_HIGH = 0.50;
float ShapeDetector::CIRCLE_BOUNDING_AREA_LOW = 0.7;
float ShapeDetector::CIRCLE_BOUNDING_AREA_HIGH = 0.8;
float ShapeDetector::CROSS_MIN_AREA = 300;
float ShapeDetector::TRI_MIN_AREA = 300;
float ShapeDetector::CIRCLE_MIN_AREA = 300;
float ShapeDetector::CROSS_ANGLE_VARIANCE = 50;
float ShapeDetector::CROSS_ANGLE_CHI = 20;

void ShapeDetector::init(ros::NodeHandle &nh) {
  nh.getParam("cross/bounding_area/low", CROSS_BOUNDING_AREA_LOW);
  nh.getParam("cross/bounding_area/high", CROSS_BOUNDING_AREA_HIGH);
  nh.getParam("triangle/bounding_area/low", TRI_BOUNDING_AREA_LOW);
  nh.getParam("triangle/bounding_area/high", TRI_BOUNDING_AREA_HIGH);
  nh.getParam("circle/bounding_area/low", CIRCLE_BOUNDING_AREA_LOW);
  nh.getParam("circle/bounding_area/high", CIRCLE_BOUNDING_AREA_HIGH);
  nh.getParam("cross/min_area", CROSS_MIN_AREA);
  nh.getParam("cross/angle/variance", CROSS_ANGLE_VARIANCE);
  nh.getParam("cross/angle/chi", CROSS_ANGLE_CHI);
  nh.getParam("triangle/min_area", TRI_MIN_AREA);
  nh.getParam("circle/min_area", CIRCLE_MIN_AREA);
}
float ShapeDetector::findAngle(cv::Point p1, cv::Point p2, cv::Point p3) {
  cv::Point v1 = p2 - p1;
  cv::Point v2 = p3 - p1;
  float m1 = sqrt(v1.x * v1.x + v1.y * v1.y);
  float m2 = sqrt(v2.x * v2.x + v2.y * v2.y);
  float thecos = v1.dot(v2) / (m1 * m2);

  return acos(thecos) * 180 / 3.1415;
}
float ShapeDetector::chisquared(std::vector<float> observed, float expected) {
  float sum = 0;
  for (int i = 0; i < observed.size(); i++) {
    sum += pow(observed[i] - expected, 2);
  }
  return sum / expected;
}

float ShapeDetector::findVariance(std::vector<float> observed) {
  float mean =
      std::accumulate(observed.begin(), observed.end(), 0.0) / observed.size();
  float sum = 0;
  for (int i = 0; i < observed.size(); i++) {
    sum += (observed[i] - mean) * (observed[i] - mean);
  }
  return sum / (observed.size() - 1);
}

bool ShapeDetector::checkBoundingAreaCross(std::vector<cv::Point> &points) {
  cv::Rect boundingRect = cv::boundingRect(points);
  float area = contourArea(points) / (boundingRect.width * boundingRect.height);
#ifdef DO_SHAPE_DEBUG
  std::cout << "Cross: Bounding area=" << area << std::endl;
#endif
  if (area >= CROSS_BOUNDING_AREA_LOW && area <= CROSS_BOUNDING_AREA_HIGH)
    return true;
  return false;
}

bool ShapeDetector::checkBoundingAreaTriangle(std::vector<cv::Point> &points) {
  cv::Rect boundingRect = cv::boundingRect(points);
  float area = contourArea(points) / (boundingRect.width * boundingRect.height);
#ifdef DO_SHAPE_DEBUG
  std::cout << "TRI: Bounding area=" << area << std::endl;
#endif
  if (area >= TRI_BOUNDING_AREA_LOW && area <= TRI_BOUNDING_AREA_HIGH)
    return true;
  return false;
}

bool ShapeDetector::checkBoundingAreaCircle(std::vector<cv::Point> &points) {
  cv::Rect boundingRect = cv::boundingRect(points);
  float area = contourArea(points) / (boundingRect.width * boundingRect.height);
#ifdef DO_SHAPE_DEBUG
  std::cout << "CIRCLE: Bounding area=" << area << std::endl;
#endif
  if (area >= CIRCLE_BOUNDING_AREA_LOW && area <= CIRCLE_BOUNDING_AREA_HIGH)
    return true;
  return false;
}

bool ShapeDetector::angleTestCross(std::vector<cv::Point> &points) {
  std::vector<float> angles;
  for (int i = 0; i < 10; i += 2) {
    angles.push_back(findAngle(points[i + 1], points[i], points[i + 2]));
  }
  angles.push_back(findAngle(points[11], points[10], points[0]));
  float chi = chisquared(angles, 90);
  float var = findVariance(angles);
#ifdef DO_SHAPE_DEBUG
  std::cout << "Cross: Chi=" << chi << " Var=" << var << std::endl;
#endif
  if (chi < CROSS_ANGLE_CHI && var < CROSS_ANGLE_VARIANCE) {
    return true;
  }
  return false;
}

bool ShapeDetector::angleTestTriangle(std::vector<cv::Point> &points) {
  std::vector<float> angles;
  angles.push_back(findAngle(points[1], points[0], points[2]));
  angles.push_back(findAngle(points[2], points[1], points[0]));
  angles.push_back(findAngle(points[0], points[1], points[2]));
  int min = 360, ind = 0;
  for (int i = 0; i < angles.size(); i++) {
    if (angles[i] < min) {
      min = angles[i];
      ind = i;
    }
  }
  int inds1, inds2;
  if (ind == 2) {
    inds1 = 0;
    inds2 = 1;
  } else if (ind == 0) {
    inds1 = 1;
    inds2 = 2;
  } else {
    inds1 = 0;
    inds2 = 2;
  }

  float v = (180 - angles[ind]) / 2;  // Not preset values, don't know the exact
                                      // angles of the triangle since not
                                      // equalterial
  float v2 = (180 - angles[inds1] - angles[inds2]);

  float chiangs = 0;
  chiangs += (angles[inds1] - v) * (angles[inds1] - v) / v;
  chiangs += (angles[inds2] - v) * (angles[inds2] - v) / v;
  chiangs += (angles[ind] - v2) * (angles[ind] - v2) / v2;
#ifdef DO_SHAPE_DEBUG
  std::cout << "TRIANGLE: Angles= [" << angles[ind] << "," << angles[inds1]
            << "," << angles[inds2] << "] Chiangs=" << chiangs << std::endl;
#endif
  if (chiangs < 1) return true;
  return false;
}

bool ShapeDetector::testRatioAreaPerimeterCircle(
    std::vector<cv::Point> &points) {
  float a = contourArea(points);
  float p = arcLength(points, true);
  float r = 4 * 3.1415 * a / (p * p);
#ifdef DO_SHAPE_DEBUG
  std::cout << "CIRCLE: Area/Perimeter=" << r << std::endl;
#endif
  if (r > 0.9 && r < 1.1) return true;
  return false;
}

bool ShapeDetector::testRatioAreaPerimeterCross(
    std::vector<cv::Point> &points) {
  float a = contourArea(points);
  float p = arcLength(points, true);
  float r = 144 / 5 * a / (p * p);
#ifdef DO_SHAPE_DEBUG
  std::cout << "CROSS: Area/Perimeter=" << r << std::endl;
#endif
  if (r > 0.9 && r < 1.1) return true;
  return false;
}

bool ShapeDetector::testPointAlignmentTriangle(std::vector<cv::Point> &points) {
  /*
  //std::cout << " --- " << std::endl;
        for(int i =0; i < points.size(); i++) {
                for(int j = 0; j < points.size(); j++) {
                        if(i != j ) {
                                if (std::abs(points[i].x - points[j].x) < 50) {
          //std::cout << "TRIANGLE: Point Alignment Failed point[" << i << "].x
  - point[" << j <<"].x = " << std::abs(points[i].x - points[j].x) << std::endl;
                                        return false;
                                }
                        }
                }
        }
        return true;
  */
  return true;
}

bool ShapeDetector::isCross(std::vector<cv::Point> &points) {
  return points.size() == 12 && contourArea(points) > CROSS_MIN_AREA &&
         testRatioAreaPerimeterCross(points) && angleTestCross(points) &&
         checkBoundingAreaCross(points);
}

bool ShapeDetector::isTriangle(std::vector<cv::Point> &points) {
  return points.size() == 3 && contourArea(points) > TRI_MIN_AREA &&
         testPointAlignmentTriangle(points) && angleTestTriangle(points) &&
         checkBoundingAreaTriangle(points);
}

bool ShapeDetector::isCircle(std::vector<cv::Point> &points) {
  return points.size() > 5 && contourArea(points) > CIRCLE_MIN_AREA &&
         testRatioAreaPerimeterCircle(points) &&
         checkBoundingAreaCircle(points);
}
