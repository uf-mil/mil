////////////////////////////////////////////////////////////
//
// AStar for occupancy grid
//
////////////////////////////////////////////////////////////
#include "AStar.h"
#include <ros/console.h>

using namespace std;


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
AStar::AStar(unsigned roi_size_) : ROI_SIZE(roi_size_), 
									whichSearch(ASTAR), 
									whichMap(EIGHT), 
									map(ROI_SIZE,std::vector<node>(ROI_SIZE))
{}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AStar::setMap(const std::vector< std::vector<bool> >  &ogridBinary)
{
	for (int row = 0; row < ROI_SIZE; ++row) {
		for (int col = 0; col < ROI_SIZE; ++col) {
			map[row][col].reset();
			if (ogridBinary[row][col]) {
				map[row][col].status = TERRAIN;
			} 
		}
	}
	setStart(ROI_SIZE/2,ROI_SIZE/2);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
vector< pair<int,int> > AStar::run()
{
			vector< pair<int,int> > list;
			ROS_INFO_STREAM("AStar: Starting from: " << startNode.x << " , " << startNode.y);
			ROS_INFO_STREAM("Astar: Going to " << endNode.x << " , " << endNode.y);

			if ( endNode.x < 0 || endNode.x >= ROI_SIZE ) { return list; } 
			if ( endNode.y < 0 || endNode.y >= ROI_SIZE ) { return list; } 
			if ( map[endNode.y][endNode.x].status == TERRAIN ) { 
				ROS_INFO_STREAM("AStar: End goal not attainable..."); 
				return list; 
			}

			priority_queue<node,vector<node>,CompareNodes> openList;
			openList.push(startNode);
			while (!openList.empty())
			{
				node nextNode = openList.top();
				//std::cout << "Checking node " << nextNode.x << "," << nextNode.y << "," << nextNode.status << std::endl;
				//if (nextNode.status == TERRAIN) { cout << "WREWRR" << endl; }
				openList.pop();
				map[nextNode.y][nextNode.x].status = CLOSED;
				if (nextNode.x == endNode.x && nextNode.y == endNode.y) {  break; } 
				update(nextNode,openList);	
			}

			//Go backwards through trajectory
			unsigned x = endNode.x;
			unsigned y = endNode.y;
			list.push_back(make_pair(x,y));
			//cout << "Creating list" << endl;
			while (true)
			{
				node nextNode = map[y][x];
				if (nextNode.parent.first < 0) {break;}
				x = nextNode.parent.first;
				y = nextNode.parent.second;
				list.push_back(make_pair(x,y));
			}
			//Remove duplicate?
			if (list.size() >= 1) { list.pop_back(); }
			ROS_INFO_STREAM("Astar: Path length is " << list.size());

			//Flip order to go from start to goal
			std::reverse(list.begin(),list.end());

			return list;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AStar::update(node &nextNode, priority_queue<node,vector<node>,CompareNodes> &openList)
{
	pair<unsigned,unsigned> neighbor;
	int neighborX,neighborY;
	double g,h;
	for (unsigned ii = 0; ii < stepSize[whichMap]; ++ii) {
		neighborX = xs[whichMap][ii]+nextNode.x;
		if ( (neighborX < 0) || (neighborX >= ROI_SIZE) ) { continue;}  
		neighborY = ys[whichMap][ii]+nextNode.y;
		if ( (neighborY < 0) || (neighborY >= ROI_SIZE) ) { continue;}  
		
		//Calculate cost function
		if (whichMap == FOUR) {
			g = nextNode.g+1;
			h = (abs(endNode.x-neighborX)+abs(endNode.y-neighborY));
		} else {
			g = nextNode.g+sqrt( pow(xs[whichMap][ii],2) + pow(ys[whichMap][ii],2) );
			int dx = neighborX-endNode.x;
			int dy = neighborY-endNode.y;
			//Chebyshev and octile (D2 = sqrt(2)) heuristic
			float D = 1, D2 = 1;
			h = D * (dx + dy) + (D2 - 2 * D) * std::min(dx, dy);

			int dx1 = neighborX - endNode.x;
			int dy1 = neighborY - endNode.y;
			int dx2 = startNode.x - endNode.x;
			int dy2 = startNode.y - endNode.y;
			double cross = abs(dx1*dy2 - dx2*dy1);
			h += cross*0.001;
			//h = sqrt(pow(endNode.x-neighborX,2)+pow(endNode.y-neighborY,2));
		}
		
		// Adjust for special tests
		if (whichSearch == DJ) {
			h = 0;
		}
		else if (whichSearch == GREEDY) {
			g = 0;
			h *= 10;
		}

		//cout << "Checking neighbor " << neighborX << "," << neighborY << " with " << status << "," << g << endl;
		//int h = 0;
		//double h = 1.0*max(fabs(endNode.x-neighborX),fabs(endNode.y-neighborY));
		unsigned status = map[neighborY][neighborX].status;
		if (status == UNKNOWN) {
			map[neighborY][neighborX].status = OPEN;
			map[neighborY][neighborX].parent = make_pair(nextNode.x,nextNode.y);
			map[neighborY][neighborX].cost(g,h);
			map[neighborY][neighborX].x = neighborX;
			map[neighborY][neighborX].y = neighborY;
			openList.push(map[neighborY][neighborX]);
		} else if (status == OPEN) {
			if (g+h < map[neighborY][neighborX].f) {
				map[neighborY][neighborX].parent = make_pair(nextNode.x,nextNode.y);
				map[neighborY][neighborX].cost(g,h);
				openList.push(map[neighborY][neighborX]);
			}
		}
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AStar::setStart(unsigned x, unsigned y)
{ 
	map[y][x].status = OPEN; 
	map[y][x].start = true;
	startNode.x = x; startNode.y = y; 
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AStar::setFinish(unsigned x, unsigned y) 
{ 
	//map[endNode.y][endNode.x].end = false;
	endNode.x = x; endNode.y = y;
	map[y][x].end = true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AStar::reset()
{
	for (auto &ii : map) {
		for (auto &jj : ii) {
			jj.reset();
		}
	}
}

