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

Eigen::Vector2d b1 (-30, 50);
Eigen::Vector2d b2 (-30, -20);
Eigen::Vector2d b3 (35, -20);
Eigen::Vector2d b4 (35, 50);



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
		if (q.size() >= 10) { q.pop_front(); }
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
	float LLA_BOUNDARY_X1 = -30, LLA_BOUNDARY_Y1 = 50;
float LLA_BOUNDARY_X2 = -30, LLA_BOUNDARY_Y2 = -20;
float LLA_BOUNDARY_X3 = 35, LLA_BOUNDARY_Y3 = -20;
float LLA_BOUNDARY_X4 = 35, LLA_BOUNDARY_Y4 = 50;
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
		#ifndef OPENCV_IRA
		void updatePointsAsCloud(const sensor_msgs::PointCloud2ConstPtr &cloud, Eigen::Affine3d T, int max_hits) 
		{
			//Decrement grid for negative persistance but only in front of the boat! This still isn't technially true since the boat 
			//can see somwhat behind itself.. THIS NEEDS TO BE FIXED!!!!
			for (int row = boatRow - ROI_SIZE/2; row < boatRow + ROI_SIZE/2; ++row) {
				for (int col = boatCol - ROI_SIZE/2; col < boatCol + ROI_SIZE/2; ++col) {
					if (ogrid[row][col].hits > 0) { 
						ogrid[row][col].hits -= 1;
						if (ogrid[row][col].hits == 0) {
							ogrid[row][col] = cell();
							//Erase 
							pointCloudTable.erase(row*GRID_SIZE+col);
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
			    Eigen::Vector2d ab = b1-b2;
			    Eigen::Vector2d ac = b1-b3;
			    Eigen::Vector2d point(xyz_in_enu(0), xyz_in_enu(1));
			    Eigen::Vector2d am = b1 - point;
			     if(ab.dot(ac) > .1){

			         ac = b1-b4;
			     }

	 
		     if(0 <= ab.dot(am) && ab.dot(am) <= ab.dot(ab) && 0 <= am.dot(ac) && am.dot(ac) <= ac.dot(ac)){
		     	//std::cout<<"TRUE"<<std::endl;
				if (xyz_in_velodyne.norm() > 5 && xyz_in_velodyne.norm() <= 100) {
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
		void setLidarPosition(geometry_msgs::Vector3 lidarPos_)
		{
			lidarPos = lidarPos_;
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
		void createBinaryROI(int minHits = 10, double maxHeight = 0)
		{
			std::fill(ogridBinary.begin(),ogridBinary.end(),std::vector<bool>(ROI_SIZE,false));
			std::fill(ogridMap.begin(),ogridMap.end(),50);
			//std::cout << "Checking row: " << boatRow - ROI_SIZE/2 << "," << boatRow + ROI_SIZE/2 << std::endl;
			//std::cout << "Checking col: " << boatCol - ROI_SIZE/2 << "," << boatCol + ROI_SIZE/2 << std::endl;
			int cnt = 0,binaryRow = 0;
			for (int row = boatRow - ROI_SIZE/2; row < boatRow + ROI_SIZE/2; ++row,++binaryRow) {
				int binaryCol = 0;
				for (int col = boatCol - ROI_SIZE/2; col < boatCol + ROI_SIZE/2; ++col,++binaryCol) {
					//if ( std::abs(ogrid[row][col].max-ogrid[row][col].min) >= heightDiff && ogrid[row][col].hits >= minHit && ogrid[row][col].max <= maxHeight) { 
					if ( ogrid[row][col].hits >= minHits && ogrid[row][col].max <= maxHeight) { 
						//std::cout << "Binary hit at " << row << "," << col << "," << (int)ogrid[row][col].hits << std::endl;
						ogridBinary[binaryRow][binaryCol] = true; 
						ogridMap[cnt] = 100;
					}
					++cnt;
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
		uint32_t boatRow = 0, boatCol = 0;						///< ???				
		geometry_msgs::Vector3 lidarPos;						///< ???
		int updateCounter = 0;									///< ???
		std::unordered_map<unsigned,beamEntry> pointCloudTable; ///< ???
};	

#endif





































/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//									Graveyard
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
		//int metersToVoxel(float d) const
		//{
		//	return floor(d/VOXEL_SIZE_METERS + GRID_SIZE/2);
		//}
					int expand = 0; 
					for (int jj = 0; jj < 9; ++jj) {
						int r = row+rOffset[jj], c = col+cOffset[jj];
						if (ogridBinary[r][c]) { ++expand; }
					}
					if (expand >= 5) {
						++expansions;
						ogridBinary[row][col] = true;
						ogridMap[row*ROI_SIZE+col] = 100;
					}

			//File saving to OpenGL
			//std::stringstream ss;
			//ss << "/home/darkknight/Code/lidarscanner/opengl/data/" << fileName << "_" << updateCounter << ".txt";
			//std::cout << "Wrinting file " << ss.str() << std::endl;
			//std::ofstream fid(ss.str());
*/