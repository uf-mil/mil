////////////////////////////////////////////////////////////
//
// Volume Classifier
//
////////////////////////////////////////////////////////////
#ifndef VOLUMECLASSIFIER_H
#define VOLUMECLASSIFIER_H

#include <vector>
#include <string>
#include "ConnectedComponents.h"
#include "navigator_msgs/PerceptionObject.h"
#include <algorithm>

void VolumeClassifier(objectMessage &object)
{
	//Seperate variables we care about
	auto h = object.maxHeightFromLidar;
	auto x = object.scale.x;
	auto y = object.scale.y;
	auto z = object.scale.z;


	//Skip classification if the object is locked, set as a start_gate, or if it isn't real
	if (object.locked || object.name == navigator_msgs::PerceptionObject::START_GATE_BUOY || !object.real) {
		return;
	}

	//Possible classifications
	std::vector<std::string> names = {"dock","shooter","scan_the_code","totem","buoy"};

	//Volume boundaries - low h, high h, low x, high x, low y, high y, low z, high z
	double volumes[5][8] = { 	{0.7,	1.25,	6.0,	7.0,	6.0,	7.0,	2.5,	3.25}, //dock (NOT TESTED!)
					{0.7,	2.0,	2.75,	4.75,	2.75,	4.75,	1.5,	3.25}, //shooter
					{0.5,	0.75,	1.9,	4.1,	1.9,	4.1,	1.9,	4.1}, //scan_the_code (needs testing!)
					{-0.6,	0.1,	1.4,	2.75,	1.4,	2.75,	1.0,	2.0}, //totems
					{-1.5,-0.75,	0.5,	1.25,	0.5,	1.25,	0.0,	1.0} }; //buoy

	//???
    for (auto ii = 0; ii < names.size(); ++ii) {
       	if (  h >= volumes[ii][0] 	&& h <= volumes[ii][1]	&& x >= volumes[ii][2] && x <= volumes[ii][3] && y >= volumes[ii][4] && y <= volumes[ii][5] && z >= volumes[ii][6]	&& z <= volumes[ii][7] )  {
           	if (object.confidence[ii] < size_t(-1) ) { ++object.confidence[ii];	}		
		}
	}

	//????
	auto match = false;
	for (auto ii = 0; ii < names.size(); ++ii) {
		if (object.confidence[ii] >= 20 ) { 
			object.name = names[ii];
			match = true;
			break;
		} 
    }

    //????
	if (!match) { 
        auto best = std::max_element(object.confidence.begin(),object.confidence.end());        
        auto dis = std::distance(object.confidence.begin(),best);
        if (*best == 0) {
            object.name = "unknown"; 
        } else {
            //ROS_INFO_STREAM("LIDAR: No match, and choosing option" << dis);
            object.name = names[dis]; 
        }
    }
    
}
#endif
