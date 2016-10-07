////////////////////////////////////////////////////////////
//
// Connected components for ROS
//
////////////////////////////////////////////////////////////
#ifndef CONNECTEDCOMPONENTS_H
#define CONNECTEDCOMPONENTS_H

#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <map>
#include <set>
#include "OccupancyGrid.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct objectStats
{
	void update(int row, int col, cell z)
	{
		//std::cout << "Stats: " << z.min << "," << z.max << std::endl;
		if (!count) {
			minRow = maxRow = row;
			minCol = maxCol = col;
			minHeight = z.min;
			maxHeight = z.max;
		} else {
			if (row < minRow) { minRow = row; }
			if (row > maxRow) { maxRow = row; }
			if (col < minCol) { minCol = col; }
			if (col > maxCol) { maxCol = col; }
			if (z.min < minHeight) { minHeight = z.min; }
			if (z.max > maxHeight) { maxHeight = z.max; }
		}
		++count;
	}
	void insert(const std::deque<LidarBeam> &newBeams) {
		//beams.insert(beams.end(),newBeams.begin(),newBeams.end());
		geometry_msgs::Point32 p32;
		for (auto beam : newBeams) {
			p32.x = beam.x;
			p32.y = beam.y;
			p32.z = beam.z;
			beams.push_back(p32);
		}
	}
	int count = 0;
	float minRow, maxRow, minCol, maxCol, minHeight, maxHeight;
	//std::vector<LidarBeam> beams;
	std::vector<geometry_msgs::Point32> beams;
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct objectMessage
{
	geometry_msgs::Point position;
	geometry_msgs::Vector3 scale;
	std::vector<geometry_msgs::Point32> beams;
	int id;
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct RCLabel
{
	RCLabel(int r, int c, int l) : row(r), col(c), label(l) {}
	int row,col,label;
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::vector< std::vector<int> > ConnectedComponents(OccupancyGrid &ogrid, std::vector<objectMessage> &objects)
{
	//std::cout << "STARTING CONNECTED COMPONENTS" << std::endl;

	std::vector<std::vector<int> > cc(ogrid.ROI_SIZE, std::vector<int>(ogrid.ROI_SIZE,0));
	const int NEIGHBORS = 4;
	int row_neighbor[NEIGHBORS] = {0,-1,1,-1};
	int col_neighbor[NEIGHBORS] = {-1,0,-1,-1};

	int label = 0;

	std::map<int,int> labelMap;

	for (int row = 1; row < ogrid.ROI_SIZE-1; ++row) {
		for (int col = 1; col < ogrid.ROI_SIZE-1; ++col) {
			//Is this a good pixel?
			if (ogrid.ogridBinary[row][col]) {
				//Check neighbors to the right and down
				std::vector<RCLabel> neighbors;
				for (int kk = 0; kk < NEIGHBORS; ++kk) {
					int nrow = row_neighbor[kk]+row;
					int ncol = col_neighbor[kk]+col;					
					//if (nrow >= 0 && ncol >= 0) {
						if ( cc[nrow][ncol] ) {
							neighbors.push_back(RCLabel(nrow,ncol,cc[nrow][ncol]));
						}
					//}
				}
				
				//Was this the first neighbor?
				if (neighbors.size() == 0) {
					++label;
					//std::cout << "New label at " << row << "," << col << " with label " << label << std::endl;
					cc[row][col] = label;
					labelMap[label] = label;
				} else {
					RCLabel min_label = *min_element(neighbors.begin(),neighbors.end(),[](RCLabel &left, RCLabel &right) { return left.label < right.label; });
					//int max_label = *max_element(neighbors.begin(),neighbors.end());
					//std::cout << "Choosing minimum label of " << min_label << " - " << max_label << " at " << row << "," << col << std::endl;
					cc[row][col] = min_label.label;
					//4 way
					////if ( labelMap.count(max_label) == 0 || labelMap[max_label] > min_label) { 
					for (auto ii : neighbors) {
						if (labelMap[ii.label] > min_label.label) {
							labelMap[ii.label] = min_label.label; 
						}
					}
				}
			}
		}
	}

	//Organize labels
	std::set<int> ids;
	for (auto ii : labelMap)  {
		int index = ii.first, match = ii.second;
		//std::cout << "old label " << ii.first << " becomes " << ii.second << std::endl;
		while (true) {
			if (labelMap[match] != match && labelMap[match] > 0) {
				labelMap[index] = labelMap[match]; match = labelMap[match];
			} else {
				break;
			}
		}
		ids.insert(match);
		//std::cout << "new label " << index << " becomes " << match << std::endl;
	}

	//Pass 2
	std::map<int,objectStats> mapObjects;
	for (int row = 0; row < ogrid.ROI_SIZE; ++row) {
		for (int col = 0; col < ogrid.ROI_SIZE; ++col) {
			if (cc[row][col]) { 
				cc[row][col] = labelMap[cc[row][col]]; 
				mapObjects[cc[row][col]].update(row,col,ogrid.ogrid[row + ogrid.boatRow - ogrid.ROI_SIZE/2][col + ogrid.boatCol - ogrid.ROI_SIZE/2]);
				int r = row + ogrid.boatRow - ogrid.ROI_SIZE/2, c = col + ogrid.boatCol - ogrid.ROI_SIZE/2;
				mapObjects[cc[row][col]].insert(ogrid.pointCloudTable[r*ogrid.GRID_SIZE+c].q);
			}
			//std::cout << cc[ii][jj] << " ";
		}
		//std::cout << std::endl;
	}

	//Re-organize obstacles, how many connections to count as an obstacle?
	int newId = 0;
	objects.clear();
	for (auto ii : mapObjects)  {
		if (ii.second.count > 1 && ii.second.beams.size() > 1) {
			objectMessage ob;
			float dx = (ii.second.maxCol-ii.second.minCol)+0.015; ob.scale.x = dx*ogrid.VOXEL_SIZE_METERS;
			float dy = (ii.second.maxRow-ii.second.minRow)+0.015; ob.scale.y = dy*ogrid.VOXEL_SIZE_METERS;
			float dz = (ii.second.maxHeight - ii.second.minHeight)+0.015; ob.scale.z = dz;
			ob.position.x = (dx/2+ii.second.minCol - ogrid.ROI_SIZE/2)*ogrid.VOXEL_SIZE_METERS + ogrid.lidarPos.x;
			ob.position.y = (dy/2+ii.second.minRow - ogrid.ROI_SIZE/2)*ogrid.VOXEL_SIZE_METERS + ogrid.lidarPos.y;
			ob.position.z =  dz/2 + ii.second.minHeight;
			ob.beams = ii.second.beams;
			objects.push_back(ob);
			//ROS_INFO_STREAM(newId << " -> " << ob.position.x << "," << ob.position.y << "," << ob.position.z << "|" << ob.scale.x << "," << ob.scale.y << "," << ob.scale.z);
			++newId;
		}
	}

	//std::cout << "FINISHED CONNECTED COMPONENTS" << std::endl;
	return cc;
}
#endif