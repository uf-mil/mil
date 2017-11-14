////////////////////////////////////////////////////////////
//
// Occupancy grid for ROS
//
////////////////////////////////////////////////////////////
#ifndef OCCUPANCYGRID_H
#define OCCUPANCYGRID_H

#include <algorithm>
#include <cmath>
#include <cstring>
#include <deque>
#include <exception>
#include <fstream>
#include <iostream>
#include <set>
#include <sstream>
#include <unordered_map>
#include <vector>
#include "lidarParams.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct LidarBeam
{
  LidarBeam() = default;
  LidarBeam(double x_, double y_, double z_, double i_, bool confident_)
    : x(x_), y(y_), z(z_), i(i_), confident(confident_)
  {
  }
  double x, y, z, i;
  bool confident;
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct cell
{
  int16_t hits = 0;
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct beamEntry
{
  void update(const LidarBeam &beam)
  {
    if (q.size() >= 10)
    {
      q.pop_front();
    }
    q.push_back(beam);
  }

  double height()
  {
    if (!q.size())
    {
      return 0;
    }
    double highZ = q[0].z, lowZ = q[0].z;
    for_each(q.begin(), q.end(), [&highZ, &lowZ](LidarBeam l) {
      highZ = std::max(l.z, highZ);
      lowZ = std::min(l.z, lowZ);
    });
    return highZ - lowZ;
  }
  std::deque<LidarBeam> q;
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
union floatConverter {
  float f;
  struct
  {
    uint8_t data[4];
  };
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class OccupancyGrid
{
public:
  ////////////////////////////////////////////////////////////
  /// \brief ?
  ///
  /// \param ?
  /// \param ?
  ////////////////////////////////////////////////////////////
  OccupancyGrid(double map_size_meters, double roi_size_meters, double voxel_size)
    : MAP_SIZE_METERS(map_size_meters)
    , VOXEL_SIZE_METERS(voxel_size)
    , GRID_SIZE(map_size_meters / voxel_size)
    , ROI_SIZE(roi_size_meters / voxel_size)
    , ogrid(GRID_SIZE, std::vector<cell>(GRID_SIZE))
    , ogridBinary(ROI_SIZE, std::vector<bool>(ROI_SIZE))
    , ogridBinaryCopy(ROI_SIZE, std::vector<bool>(ROI_SIZE))
    , ogridMap(std::vector<int8_t>(ROI_SIZE * ROI_SIZE))
  {
    // std::cout << ogrid.size() << "," << ogrid[0].size() << std::endl;
  }

  ////////////////////////////////////////////////////////////
  /// \brief ?
  ///
  /// \param ?
  /// \param ?
  ////////////////////////////////////////////////////////////
  void reset()
  {
    std::fill(ogrid.begin(), ogrid.end(), std::vector<cell>(GRID_SIZE, cell()));
    std::fill(ogridBinary.begin(), ogridBinary.end(), std::vector<bool>(ROI_SIZE, false));
    std::fill(ogridMap.begin(), ogridMap.end(), 50);
    pointCloudTable_Uno.clear();
    pointCloudTable.clear();
  }

  ////////////////////////////////////////////////////////////
  /// \brief ?
  ///
  /// \param ?
  /// \param ?
  ////////////////////////////////////////////////////////////
  void updatePoints(const std::vector<LidarBeam> &xyz, int max_hits)
  {
    for (auto p : xyz)
      updateGrid(p, max_hits);
  }

  ////////////////////////////////////////////////////////////
  /// \brief ?
  ///
  /// \param ?
  /// \param ?
  ////////////////////////////////////////////////////////////
  void setBoundingBox(Eigen::Vector2d b1, Eigen::Vector2d b2, Eigen::Vector2d b3, Eigen::Vector2d b4)
  {
    this->b1 = b1;
    ab = b1 - b2;
    ac = b1 - b3;
    // Why .1 for the dot product result?
    if (ab.dot(ac) > .1)
    {
      ac = b1 - b4;
    }
  }

////////////////////////////////////////////////////////////
/// \brief ?
///
/// \param ?
/// \param ?
////////////////////////////////////////////////////////////
#ifndef OPENCV_IRA
  void updatePointsAsCloud(const sensor_msgs::PointCloud2ConstPtr &cloud, Eigen::Affine3d T, int max_hits,
                           double MAXIMUM_Z_BELOW_LIDAR, double MAXIMUM_Z_ABOVE_LIDAR)
  {
    // Reset point cloud table uno
    pointCloudTable_Uno.clear();

    // Decrement area around boat we have lidar confidence in
    int scanDistanceHalf = LIDAR_VIEW_DISTANCE_METERS / VOXEL_SIZE_METERS;
    double colMin = ((boatCol - ROI_SIZE / 2) - GRID_SIZE / 2) * VOXEL_SIZE_METERS,
           colMax = ((boatCol + ROI_SIZE / 2) - GRID_SIZE / 2) * VOXEL_SIZE_METERS;
    double rowMin = ((boatRow - ROI_SIZE / 2) - GRID_SIZE / 2) * VOXEL_SIZE_METERS,
           rowMax = ((boatRow + ROI_SIZE / 2) - GRID_SIZE / 2) * VOXEL_SIZE_METERS;
    // std::cout << "Scanning around boat x,y " << colMin << "," << colMax << " and " << rowMin << "," << rowMax <<
    // std::endl;
    for (int row = boatRow - scanDistanceHalf; row < boatRow + scanDistanceHalf; ++row)
    {
      for (int col = boatCol - scanDistanceHalf; col < boatCol + scanDistanceHalf; ++col)
      {
        Eigen::Vector3d dir(col - boatCol, row - boatRow, 0);
        dir.normalize();
        double lidarAngle = fabs(acos(lidarHeading.dot(dir)) * 180. / M_PI);
        double distance = sqrt(pow(boatRow - row, 2) + pow(boatCol - col, 2)) * VOXEL_SIZE_METERS;
        // std::cout << row << " , " << col << " , " << lidarAngle << " , " << distance << std::endl;
        try
        {
          auto val = ogrid.at(row).at(col).hits;
          if (lidarAngle <= LIDAR_VIEW_ANGLE_DEG && val > 0 && distance >= LIDAR_MIN_VIEW_DISTANCE_METERS)
          {
            ogrid[row][col].hits -= 1;
            if (ogrid[row][col].hits < 0)
            {
              ogrid[row][col] = cell();
            }
          }
        }
        catch (std::exception &e)
        {
          std::cout << "OUT OF BOUNDS" << row << col << std::endl;
          std::cout << e.what() << std::endl;
        }
      }
    }

    // Raw Step through data
    for (auto ii = 0, jj = 0; ii < cloud->width; ++ii, jj += cloud->point_step)
    {
      floatConverter x, y, z, i;
      for (int kk = 0; kk < 4; ++kk)
      {
        x.data[kk] = cloud->data[jj + kk];
        y.data[kk] = cloud->data[jj + 4 + kk];
        z.data[kk] = cloud->data[jj + 8 + kk];
        i.data[kk] = cloud->data[jj + 16 + kk];
      }
      Eigen::Vector3d xyz_in_velodyne(x.f, y.f, z.f);
      Eigen::Vector3d xyz_in_enu = T * xyz_in_velodyne;
      Eigen::Vector2d point(xyz_in_enu(0), xyz_in_enu(1));
      Eigen::Vector2d am = b1 - point;
      // Valid lidar points are inside bounding box or within X meters of the boat
      if ((0 <= ab.dot(am) && ab.dot(am) <= ab.dot(ab) && 0 <= am.dot(ac) && am.dot(ac) <= ac.dot(ac)) ||
          (xyz_in_velodyne.norm() <= 15))
      {
        // std::cout << "THIS POINT IS in boudds!" <<std::endl;
        if (xyz_in_velodyne.norm() >= LIDAR_MIN_VIEW_DISTANCE_METERS && xyz_in_velodyne.norm() <= 100 &&
            xyz_in_enu(2) >= lidarPos.z - MAXIMUM_Z_BELOW_LIDAR && xyz_in_enu(2) <= lidarPos.z + MAXIMUM_Z_ABOVE_LIDAR)
        {
          // std::cout << "fuck!" <<std::endl;
          auto valid = xyz_in_velodyne.norm() < LIDAR_CONFIDENCE_DISTANCE_METERS && goodLidarReading == true;
          updateGrid(LidarBeam(xyz_in_enu(0), xyz_in_enu(1), xyz_in_enu(2), i.f, valid), max_hits);
        }
      }
      else
      {
        std::cout << "THIS POINT IS OUT OF THE BOUNDS!" << std::endl;
      }
    }
    ++updateCounter;
  }
#endif

  ////////////////////////////////////////////////////////////
  /// \brief ?
  ///
  /// \param ?
  /// \param ?
  ////////////////////////////////////////////////////////////
  void setLidarPosition(geometry_msgs::Vector3 lidarPos_, Eigen::Vector3d lidarHeading_)
  {
    lidarPos = lidarPos_;
    lidarHeading = lidarHeading_;
    boatRow = floor(lidarPos.y / VOXEL_SIZE_METERS + GRID_SIZE / 2);
    boatCol = floor(lidarPos.x / VOXEL_SIZE_METERS + GRID_SIZE / 2);
  }

  ////////////////////////////////////////////////////////////
  /// \brief ?
  ///
  /// \param ?
  /// \param ?
  ////////////////////////////////////////////////////////////
  float ROItoMeters(int voxel) const
  {
    return (voxel - ROI_SIZE / 2) * VOXEL_SIZE_METERS;
  }

  ////////////////////////////////////////////////////////////
  /// \brief ?
  ///
  /// \param ?
  /// \param ?
  ////////////////////////////////////////////////////////////
  void updateGrid(LidarBeam p, int max_hits)
  {
    int x = floor(p.x / VOXEL_SIZE_METERS + GRID_SIZE / 2);
    int y = floor(p.y / VOXEL_SIZE_METERS + GRID_SIZE / 2);
    if (x >= 0 && x < GRID_SIZE && y >= 0 && y < GRID_SIZE)
    {
      ogrid[y][x].hits += LIDAR_HITS_INCREMENT;
      pointCloudTable[y * GRID_SIZE + x].update(p);
      pointCloudTable_Uno[y * GRID_SIZE + x].push_back(p);
      if (ogrid[y][x].hits > max_hits)
      {
        ogrid[y][x].hits = max_hits;
      }
    }
  }

  ////////////////////////////////////////////////////////////
  /// \brief ?
  ///
  /// \param ?
  /// \param ?
  ////////////////////////////////////////////////////////////
  void validateOrientation(Eigen::Matrix3d mat)
  {
    goodLidarReading = true;
    double ang[3];
    ang[0] = atan2(-mat(1, 2), mat(2, 2));
    double sr = sin(ang[0]), cr = cos(ang[0]);
    ang[1] = atan2(mat(0, 2), cr * mat(2, 2) - sr * mat(1, 2));
    ang[2] = atan2(-mat(0, 1), mat(0, 0));
    ROS_INFO_STREAM("LIDAR | BOAT XYZ Rotation: " << ang[0] * 180 / M_PI << "," << ang[1] * 180 / M_PI << ","
                                                  << ang[2] * 180 / M_PI);
    if (fabs(ang[0] * 180 / M_PI) > MAX_ROLL_PITCH_ANGLE_DEG || fabs(ang[1] * 180 / M_PI) > MAX_ROLL_PITCH_ANGLE_DEG)
    {
      // BOOST_ASSERT_MSG(fabs(false, "BOAT IN POOR ROLL_PITCH");
      goodLidarReading = false;
    }
  }

  ////////////////////////////////////////////////////////////
  /// \brief ?
  ///
  /// \param ?
  /// \param ?
  ////////////////////////////////////////////////////////////
  void createBinaryROI(int minHits = 10)
  {
    std::fill(ogridBinary.begin(), ogridBinary.end(), std::vector<bool>(ROI_SIZE, false));
    std::fill(ogridMap.begin(), ogridMap.end(), 50);
    double colMin = ((boatCol - ROI_SIZE / 2) - GRID_SIZE / 2) * VOXEL_SIZE_METERS,
           colMax = ((boatCol + ROI_SIZE / 2) - GRID_SIZE / 2) * VOXEL_SIZE_METERS;
    double rowMin = ((boatRow - ROI_SIZE / 2) - GRID_SIZE / 2) * VOXEL_SIZE_METERS,
           rowMax = ((boatRow + ROI_SIZE / 2) - GRID_SIZE / 2) * VOXEL_SIZE_METERS;
    int binaryRow = 0;
    for (int row = boatRow - ROI_SIZE / 2; row < boatRow + ROI_SIZE / 2; ++row, ++binaryRow)
    {
      int binaryCol = 0;
      for (int col = boatCol - ROI_SIZE / 2; col < boatCol + ROI_SIZE / 2; ++col, ++binaryCol)
      {
        try
        {
          if (ogrid.at(row).at(col).hits >= minHits &&
              pointCloudTable.at(row * GRID_SIZE + col).q.size() >= MIN_LIDAR_POINTS_FOR_OCCUPANCY &&
              pointCloudTable.at(row * GRID_SIZE + col).height() >= MIN_OBJECT_HEIGHT_METERS)
          {
            ogridBinary[binaryRow][binaryCol] = true;
            ogridMap[binaryRow * ROI_SIZE + binaryCol] = 100;
          }
        }
        catch (std::exception &e)
        {
          std::cout << e.what() << __PRETTY_FUNCTION__ << std::endl;
        }
      }
    }
  }

  ////////////////////////////////////////////////////////////
  /// \brief ?
  ///
  /// \param ?
  /// \param ?
  ////////////////////////////////////////////////////////////
  void inflateBinary(int inflateSize = 2)
  {
    ogridBinaryCopy = ogridBinary;
    int rOffset[] = { -1, -1, -1, 0, 0, 0, 1, 1, 1 };
    int cOffset[] = { -1, 0, 1, -1, 0, 1, -1, 0, 1 };
    for (int row = inflateSize; row < ROI_SIZE - inflateSize - 1; ++row)
    {
      for (int col = inflateSize; col < ROI_SIZE - inflateSize - 1; ++col)
      {
        if (ogridBinaryCopy[row][col])
        {
          for (int kk = 1; kk <= inflateSize; ++kk)
          {
            for (int jj = 0; jj < 9; ++jj)
            {
              int r = row + rOffset[jj] * kk, c = col + cOffset[jj] * kk;
              ogridBinary[r][c] = true;
              ogridMap[r * ROI_SIZE + c] = 100;
            }
          }
        }
      }
    }
  }

  ////////////////////////////////////////////////////////////
  /// \brief ?
  ///
  /// \param ?
  /// \param ?
  ////////////////////////////////////////////////////////////
  void deflateBinary(int deflateSize = 2)
  {
    for (int ii = 0; ii < deflateSize; ++ii)
    {
      ogridBinaryCopy = ogridBinary;
      int rOffset[] = { -1, 0, 0, 1 };
      int cOffset[] = { 0, -1, 1, 0 };
      for (int row = 1; row < ROI_SIZE - 1; ++row)
      {
        for (int col = 1; col < ROI_SIZE - 1; ++col)
        {
          if (ogridBinaryCopy[row][col])
          {
            for (int jj = 0; jj < 4; ++jj)
            {
              int r = row + rOffset[jj], c = col + cOffset[jj];
              if (ogridBinaryCopy[r][c] == false)
              {
                ogridMap[row * ROI_SIZE + col] = 50;
              }
            }
          }
        }
      }
    }
  }

public:
  ////////////////////////////////////////////////////////////
  // Public Member data
  ////////////////////////////////////////////////////////////
  const double MAP_SIZE_METERS, VOXEL_SIZE_METERS;  ///< ???
  const int GRID_SIZE, ROI_SIZE;                    ///< ???
  std::vector<std::vector<cell>> ogrid;             ///< ???
  std::vector<std::vector<bool>> ogridBinary;       ///< ???
  std::vector<std::vector<bool>> ogridBinaryCopy;   ///< ???
  std::vector<int8_t> ogridMap;                     ///< ???
  int boatRow = 0, boatCol = 0;                     ///< ???
  geometry_msgs::Vector3 lidarPos;                  ///< ???
  Eigen::Vector3d lidarHeading;
  int updateCounter = 0;                                                     ///< ???
  std::unordered_map<unsigned, beamEntry> pointCloudTable;                   ///< ???
  std::unordered_map<unsigned, std::vector<LidarBeam>> pointCloudTable_Uno;  ///< ???
  Eigen::Vector2d b1, ab, ac;                                                ///< ???
  bool goodLidarReading = true;
};

#endif
