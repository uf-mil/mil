////////////////////////////////////////////////////////////
//
// AStar for occupancy grid
//
////////////////////////////////////////////////////////////
#ifndef NODE_H
#define NODE_H

#include <algorithm>

enum statusTypes {UNKNOWN,OPEN,CLOSED,TERRAIN};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct point
{
	double x,y;
	void unit() { double mag = sqrt(x*x+y*y); x/=mag; y/=mag; }
};


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class node
{
	public:
		node() : parent(std::make_pair(-1,-1)), x(0), y(0), status(UNKNOWN), f(0), g(0), h(0) {}
		
		node(unsigned x_, unsigned y_) :  parent(std::make_pair(-1,-1)), x(x_), y(y_), status(UNKNOWN), f(0), g(0), h(0) {}
		
		void cost(double g_, double h_) { g = g_; h = h_; f = g+h;}
		void reset() 
		{ 
			parent = std::make_pair(-1,-1);
			g = f = h = 0; 
			status = UNKNOWN;
		}

		std::pair<int, int> parent;
		int x,y;
		statusTypes status;	
		double f,g,h;
		bool start = false;
		bool end = false;
		//static std::vector< std::vector<node> > *map;
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct CompareNodes
{
	public:
		bool operator() (const node &lhs, const node &rhs)
		{
			return (lhs.f > rhs.f);
		}
};

#endif














































/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//									Graveyard
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
class compareNode
{
	public:
		bool operator() (const node &lhs, const node &rhs)
		{
			if (lhs.f == rhs.f) 
			{ 
				// left
				node momL, grandmaL;
				if (lhs.parent.first != -1)
				{
					std::cout << "Checking parent 1 at " << lhs.parent.first << "," << lhs.parent.second << std::endl;
					//std::cout << "Size of map " << node::map.size() << std::endl;
					momL = (*node::map)[lhs.parent.second][lhs.parent.first];
				}	
				if (momL.parent.first != -1)
				{
					std::cout << "Checking parent 2 at " << momL.parent.first << "," << momL.parent.second << std::endl;
					//std::cout << "Size of map " << node::map.size() << std::endl;					
					grandmaL = (*node::map)[momL.parent.second][momL.parent.first];
				
				
					point v1{static_cast<double>(momL.x-grandmaL.x),static_cast<double>(momL.y-grandmaL.y)};
					v1.unit();
					point v2{static_cast<double>(lhs.x-momL.x),static_cast<double>(lhs.y-momL.y)};
					v2.unit();
					
					std::cout << "v1: " << v1.x << "," << v1.y << std::endl;
					std::cout << "v2: " << v2.x << "," << v2.y << std::endl;
					if (v1.x == v2.x && v1.y == v2.y) { return false; }
					else { return true; }
				}
			}
			return (lhs.f > rhs.f);
		}
};
*/