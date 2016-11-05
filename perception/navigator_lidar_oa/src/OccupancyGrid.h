////////////////////////////////////////////////////////////
//
// Occupancy grid for ROS
//
////////////////////////////////////////////////////////////
#ifndef OCCUPANCYGRID_H
#define OCCUPANCYGRID_H

#include <iostream>
#include <cmath>
#include <vector>
#include <unordered_map>
#include <cstring>
#include <deque>
#include <set>
#include <algorithm>
#include <fstream>
#include <sstream>


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct LidarBeam
{
	LidarBeam() = default;
	LidarBeam(double x_, double y_, double z_, double i_) : x(x_),y(y_),z(z_),i(i_) {}
	double x,y,z,i;
};


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct cell
{
	int16_t hits = 0;
	float min = 1e5,max = -1e5;
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct beamEntry
{
	void update(const LidarBeam &beam) {
		if (q.size() >= 30) { q.pop_front(); }
		q.push_back(beam);
	}
	std::deque<LidarBeam> q;
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
union floatConverter
{
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
		OccupancyGrid(double map_size_meters, double roi_size_meters, double voxel_size) : MAP_SIZE_METERS(map_size_meters), 
								    VOXEL_SIZE_METERS(voxel_size),
								    GRID_SIZE(map_size_meters/voxel_size),
								    ROI_SIZE(roi_size_meters/voxel_size),
								    ogrid(GRID_SIZE, std::vector<cell>(GRID_SIZE)),
								    ogridBinary(ROI_SIZE, std::vector<bool>(ROI_SIZE)),
								    ogridBinaryCopy(ROI_SIZE, std::vector<bool>(ROI_SIZE)),
								    ogridMap(std::vector<int8_t>(ROI_SIZE*ROI_SIZE))								 
		{
			//std::cout << ogrid.size() << "," << ogrid[0].size() << std::endl;
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
				updateGrid(p,max_hits);
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
			ab = b1-b2;
			ac = b1-b3;		
			//Why .1 for the dot product result?
			if(ab.dot(ac) > .1) {
		         ac = b1-b4;
		     }	
		}

		////////////////////////////////////////////////////////////
	    /// \brief ?
	    ///
	    /// \param ?
	    /// \param ?
	    ////////////////////////////////////////////////////////////
		void setLidarParams(const double LIDAR_VIEW_ANGLE_DEG, const double LIDAR_VIEW_DISTANCE_METERS, const double LIDAR_MIN_VIEW_DISTANCE, const int MIN_LIDAR_POINTS_FOR_OCCUPANCY, const double MIN_OBJECT_HEIGHT_METERS)
		{
			lidarViewAngle = LIDAR_VIEW_ANGLE_DEG;
			lidarViewDistance = LIDAR_VIEW_DISTANCE_METERS;
            lidarMinViewDistance = LIDAR_MIN_VIEW_DISTANCE;
            lidarMinPoints = MIN_LIDAR_POINTS_FOR_OCCUPANCY;
            objectMinHeight = MIN_OBJECT_HEIGHT_METERS;
		}


		////////////////////////////////////////////////////////////
	    /// \brief ?
	    ///
	    /// \param ?
	    /// \param ?
	    ////////////////////////////////////////////////////////////
		#ifndef OPENCV_IRA
		void updatePointsAsCloud(const sensor_msgs::PointCloud2ConstPtr &cloud, Eigen::Affine3d T, int max_hits, double MAXIMUM_Z_BELOW_LIDAR, double MAXIMUM_Z_ABOVE_LIDAR) 
		{
			//Reset point cloud table uno
			pointCloudTable_Uno.clear();

			//Decrement area around boat we have lidar confidence in
			int scanDistanceHalf = lidarViewDistance/VOXEL_SIZE_METERS;
			double colMin = ((boatCol-ROI_SIZE/2) - GRID_SIZE/2)*VOXEL_SIZE_METERS, colMax = ((boatCol+ROI_SIZE/2) - GRID_SIZE/2)*VOXEL_SIZE_METERS;
			double rowMin = ((boatRow-ROI_SIZE/2) - GRID_SIZE/2)*VOXEL_SIZE_METERS, rowMax = ((boatRow+ROI_SIZE/2) - GRID_SIZE/2)*VOXEL_SIZE_METERS;
			//std::cout << "Scanning around boat x,y " << colMin << "," << colMax << " and " << rowMin << "," << rowMax << std::endl;
			for (int row = boatRow - scanDistanceHalf; row < boatRow + scanDistanceHalf; ++row) {
				for (int col = boatCol - scanDistanceHalf; col < boatCol + scanDistanceHalf; ++col) {
					Eigen::Vector3d dir(col-boatCol,row-boatRow,0); dir.normalize();
					double lidarAngle = fabs(acos(lidarHeading.dot(dir))*180./M_PI);
					double distance = sqrt(pow(boatRow - row, 2) + pow(boatCol - col, 2)) * VOXEL_SIZE_METERS;
					//std::cout << row << " , " << col << " , " << lidarAngle << " , " << distance << std::endl;
					if (lidarAngle <= lidarViewAngle && ogrid[row][col].hits > 0 && distance >= lidarMinViewDistance) { 
						ogrid[row][col].hits -= 1;
						if (ogrid[row][col].hits == 0) {
							ogrid[row][col] = cell();
							pointCloudTable[row*GRID_SIZE+col].q.clear();
						}	
					} 
				}
			}
			
			//Raw Step through data
			for (auto ii = 0, jj = 0; ii < cloud->width; ++ii,jj+=cloud->point_step) {
				floatConverter x,y,z,i;
				for (int kk = 0; kk < 4; ++kk)
				{
					x.data[kk] = cloud->data[jj+kk];
					y.data[kk] = cloud->data[jj+4+kk];
					z.data[kk] = cloud->data[jj+8+kk];
					i.data[kk] = cloud->data[jj+16+kk];
				}
				Eigen::Vector3d xyz_in_velodyne(x.f,y.f,z.f);
				Eigen::Vector3d xyz_in_enu = T*xyz_in_velodyne;
			    Eigen::Vector2d point(xyz_in_enu(0), xyz_in_enu(1));
			    Eigen::Vector2d am = b1 - point;
		     	if(0 <= ab.dot(am) && ab.dot(am) <= ab.dot(ab) && 0 <= am.dot(ac) && am.dot(ac) <= ac.dot(ac)){
		     		//std::cout<<"TRUE"<<std::endl;
					if (xyz_in_velodyne.norm() >= 1 && xyz_in_velodyne.norm() <= 75 && xyz_in_enu(2) >= lidarPos.z-MAXIMUM_Z_BELOW_LIDAR && xyz_in_enu(2) <= lidarPos.z+MAXIMUM_Z_ABOVE_LIDAR) {
						updateGrid(LidarBeam(xyz_in_enu(0), xyz_in_enu(1), xyz_in_enu(2),i.f),max_hits);
					}
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
			boatRow = floor(lidarPos.y/VOXEL_SIZE_METERS + GRID_SIZE/2);
			boatCol = floor(lidarPos.x/VOXEL_SIZE_METERS + GRID_SIZE/2);
		}

	    ////////////////////////////////////////////////////////////
	    /// \brief ?
	    ///
	    /// \param ?
	    /// \param ?
	    ////////////////////////////////////////////////////////////
		float ROItoMeters(int voxel) const
		{
			return (voxel - ROI_SIZE/2)*VOXEL_SIZE_METERS;
		}

	    ////////////////////////////////////////////////////////////
	    /// \brief ?
	    ///
	    /// \param ?
	    /// \param ?
	    ////////////////////////////////////////////////////////////
		void updateGrid(LidarBeam p, int max_hits)
		{
				int x = floor(p.x/VOXEL_SIZE_METERS + GRID_SIZE/2);
				int y = floor(p.y/VOXEL_SIZE_METERS + GRID_SIZE/2);
				float z = p.z;
				if (x >= 0 && x < GRID_SIZE && y >= 0 && y < GRID_SIZE) {
					if (z < ogrid[y][x].min) { ogrid[y][x].min = z; }
					if (z > ogrid[y][x].max) { ogrid[y][x].max = z; }
					ogrid[y][x].hits += 5;
					pointCloudTable[y*GRID_SIZE+x].update(p);
					pointCloudTable_Uno[y*GRID_SIZE+x].push_back(p);
					if (ogrid[y][x].hits > max_hits) { 
						//std::cout << "Ogrid at y,x " << y << "," << x << " has hits of " << (int)ogrid[y][x].hits << std::endl;	
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
		void createBinaryROI(int minHits = 10)
		{
			std::fill(ogridBinary.begin(),ogridBinary.end(),std::vector<bool>(ROI_SIZE,false));
			std::fill(ogridMap.begin(),ogridMap.end(),50);
			double colMin = ((boatCol-ROI_SIZE/2) - GRID_SIZE/2)*VOXEL_SIZE_METERS, colMax = ((boatCol+ROI_SIZE/2) - GRID_SIZE/2)*VOXEL_SIZE_METERS;
			double rowMin = ((boatRow-ROI_SIZE/2) - GRID_SIZE/2)*VOXEL_SIZE_METERS, rowMax = ((boatRow+ROI_SIZE/2) - GRID_SIZE/2)*VOXEL_SIZE_METERS;
			//std::cout << "ROI around boat x,y " << colMin << "," << colMax << " and " << rowMin << "," << rowMax << std::endl;
			int binaryRow = 0;
			for (int row = boatRow - ROI_SIZE/2; row < boatRow + ROI_SIZE/2; ++row,++binaryRow) {
				int binaryCol = 0;
				for (int col = boatCol - ROI_SIZE/2; col < boatCol + ROI_SIZE/2; ++col,++binaryCol) {
					//if ( std::abs(ogrid[row][col].max-ogrid[row][col].min) >= heightDiff && ogrid[row][col].hits >= minHit && ogrid[row][col].max <= maxHeight) { 
					if ( ogrid[row][col].hits >= minHits && pointCloudTable[row*GRID_SIZE+col].q.size() >= lidarMinPoints && ogrid[row][col].max-ogrid[row][col].min >= objectMinHeight) { 
						//std::cout << "Found " << row << "," << col << " has " << pointCloudTable[row*GRID_SIZE+col].q.size() << " points" << std::endl;
						//double r_enu = (row - GRID_SIZE/2)*VOXEL_SIZE_METERS,c_enu = (col - GRID_SIZE/2)*VOXEL_SIZE_METERS;
						//std::cout << "Binary hit at x,y " << c_enu << "," << r_enu << "," << ogrid[row][col].hits << std::endl;
						//std::cout << "which is " << binaryRow*ROI_SIZE+binaryCol << std::endl;
						ogridBinary[binaryRow][binaryCol] = true; 
						ogridMap[binaryRow*ROI_SIZE+binaryCol] = 100;
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
		void inflateBinary(int inflateSize = 3)
		{
			ogridBinaryCopy = ogridBinary;
			int rOffset[] = {-1,-1,-1,0,0,0,1,1,1};
			int cOffset[] = {-1,0,1,-1,0,1,-1,0,1};
			for (int row = inflateSize; row < ROI_SIZE-inflateSize-1; ++row) {
				for (int col = inflateSize; col < ROI_SIZE-inflateSize-1; ++col) {
					if (ogridBinaryCopy[row][col]) {
						//std::cout << "Now " << row << "," << col << " has " << pointCloudTable[(row+boatRow-ROI_SIZE/2)*GRID_SIZE+(col+boatCol-ROI_SIZE/2)].q.size() << " points" << std::endl;
						//double r_enu = (row + boatRow-ROI_SIZE/2 - GRID_SIZE/2)*VOXEL_SIZE_METERS,c_enu = (col + boatCol-ROI_SIZE/2 - GRID_SIZE/2)*VOXEL_SIZE_METERS;
						//std::cout << "Inflation hit at x,y (" << col << "," << row << ") " << c_enu << "," << r_enu << "," << ogrid[row][col].hits << std::endl;	
						for (int kk = 1; kk <= inflateSize; ++kk) {
							for (int jj = 0; jj < 9; ++jj) {
								int r = row+rOffset[jj]*kk, c = col+cOffset[jj]*kk;							
								ogridBinary[r][c] = true;
								ogridMap[r*ROI_SIZE+c] = 100;
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
		const double MAP_SIZE_METERS, VOXEL_SIZE_METERS; 		///< ???
		const int GRID_SIZE, ROI_SIZE; 							///< ???
		std::vector< std::vector<cell> > ogrid;					///< ???
		std::vector< std::vector<bool> >  ogridBinary;			///< ???
		std::vector< std::vector<bool> >  ogridBinaryCopy;		///< ???
		std::vector< int8_t > ogridMap;							///< ???
		int boatRow = 0, boatCol = 0;						///< ???				
		geometry_msgs::Vector3 lidarPos;						///< ???
		Eigen::Vector3d lidarHeading;
		int updateCounter = 0;									///< ???
		std::unordered_map<unsigned,beamEntry> pointCloudTable; ///< ???
		std::unordered_map<unsigned,std::vector<LidarBeam>> pointCloudTable_Uno; ///< ???
		Eigen::Vector2d b1,ab,ac;									///< ???
		double lidarViewAngle,lidarViewDistance,lidarMinViewDistance,objectMinHeight;
		int lidarMinPoints;
};	

#endif

/*
//Notes
#include <sensor_msgs/point_cloud2_iterator.h>
  sensor_msgs::PointCloud2Iterator<float> iter_x{output_pcd, "x"};
  sensor_msgs::PointCloud2Iterator<float> iter_y{output_pcd, "y"};
  sensor_msgs::PointCloud2Iterator<float> iter_z{output_pcd, "z"};
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(output_pcd, "b");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(output_pcd, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(output_pcd, "r");

rosrun navigator_perception stereo_point_cloud_driver
	rosservice call /stereo/activation_srv "data: true" 

*/
