#include "ConnectedComponents.h"
#include "navigator_msgs/PerceptionObject.h"

void VolumeClassifier(objectMessage &object)
{
	//If we classify something as shooter/scanthecode, leave it alone
	if (object.locked || object.name == navigator_msgs::PerceptionObject::START_GATE_BUOY || !object.real) {
		return;
	}
	//THESE VAlUES WILL ALL BE ADJUSTED ON NEXT LAKE DAY!
	if (object.maxHeightFromLidar >= 1.25 && object.scale.x > 3 && object.scale.y > 3 && object.scale.z > 1.25) {
		object.name = navigator_msgs::PerceptionObject::DETECT_DELIVER_PLATFORM;
	} else if (object.maxHeightFromLidar >= 0.75 && object.scale.x > 2 && object.scale.y > 2 && object.scale.z > 1.25) {
		object.name = navigator_msgs::PerceptionObject::SCAN_THE_CODE;
	} else if (object.maxHeightFromLidar >= -0.25 && object.scale.x > 1.2 && object.scale.y > 1.2 && object.scale.z > 1.2) {
		object.name = navigator_msgs::PerceptionObject::TOTEM;
	} else if (object.maxHeightFromLidar < -0.25 && object.scale.x <= 1.2 && object.scale.y <= 1.2) {
		object.name = navigator_msgs::PerceptionObject::BUOY;
	} else {
		object.name = navigator_msgs::PerceptionObject::UNKNOWN;
	}
}
