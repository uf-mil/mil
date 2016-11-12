////////////////////////////////////////////////////////////
//
// Volume Classifier
//
////////////////////////////////////////////////////////////
#ifndef VOLUMECLASSIFIER_H
#define VOLUMECLASSIFIER_H

#include "ConnectedComponents.h"
#include "navigator_msgs/PerceptionObject.h"

void VolumeClassifier(objectMessage &object)
{
	//If we classify something as shooter/scanthecode, leave it alone
	if (object.locked || object.name == navigator_msgs::PerceptionObject::START_GATE_BUOY || !object.real) {
		return;
	}
	//THESE VAlUES WILL ALL BE ADJUSTED ON NEXT LAKE DAY!
	if ( (object.maxHeightFromLidar >= 1.25 && object.scale.x > 7 && object.scale.y > 7 && object.scale.z > 1.25) || object.name == navigator_msgs::PerceptionObject::IDENTIFY_AND_DOCK ) {
		object.name = navigator_msgs::PerceptionObject::IDENTIFY_AND_DOCK;
	} else if ( (object.maxHeightFromLidar >= 1.25 && object.scale.x > 4 && object.scale.y > 4 && object.scale.z > 1.25) || object.name == navigator_msgs::PerceptionObject::DETECT_DELIVER_PLATFORM ) {
		object.name = navigator_msgs::PerceptionObject::DETECT_DELIVER_PLATFORM;
	} else if ( (object.maxHeightFromLidar >= 0.75 && object.scale.x > 2 && object.scale.y > 2 && object.scale.z > 1.25) || object.name == navigator_msgs::PerceptionObject::SCAN_THE_CODE) {
		object.name = navigator_msgs::PerceptionObject::SCAN_THE_CODE;
	} else if ( (object.maxHeightFromLidar >= -0.25 && object.scale.x > 1.2 && object.scale.y > 1.2 && object.scale.z > 1.2) || object.name == navigator_msgs::PerceptionObject::TOTEM ) {
		object.name = navigator_msgs::PerceptionObject::TOTEM;
	} else if ( (object.maxHeightFromLidar < -0.25 && object.scale.x <= 1.2 && object.scale.y <= 1.2) ||  object.name == navigator_msgs::PerceptionObject::BUOY) {
		object.name = navigator_msgs::PerceptionObject::BUOY;
	} else {
		object.name = navigator_msgs::PerceptionObject::UNKNOWN;
	}
}
#endif