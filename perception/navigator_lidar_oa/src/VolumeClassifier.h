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
					{0.9,	2.0,	2.5,	4.0,	2.5,	4.0,	2.5,	3.5}, //shooter
					{0.2,	0.75,	1.3,	2.25,	1.3,	2.25,	1.7,	2.5}, //scan_the_code
					{-0.6,	0.1,	0.8,	1.8,	0.8,	1.8,	0.8,	1.8}, //totems
					{-1.25,-0.8,	0.125,	1.0,	0.125,	1.0,	0.125,	1.0} }; //buoy

	//???
    for (auto ii = 0; ii < names.size(); ++ii) {
       	if (  h >= volumes[ii][0] 	&& h <= volumes[ii][1]	&& ( (x >= volumes[ii][2] && x <= volumes[ii][3]) ||  (y >= volumes[ii][4] && y <= volumes[ii][5]) ) && z >= volumes[ii][6]	&& z <= volumes[ii][7] )  {
           	if (object.confidence[ii] < size_t(-1) ) { ++object.confidence[ii];	}		
		}
	}

	//????
	auto match = false;
	for (auto ii = 0; ii < names.size(); ++ii) {
		if (object.confidence[ii] >= 31 ) { 
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
            object.name = names[dis]; 
        }
    }
    
}
#endif
