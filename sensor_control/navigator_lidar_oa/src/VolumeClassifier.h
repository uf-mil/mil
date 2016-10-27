#include "ConnectedComponents.h"
#include "navigator_msgs/PerceptionObject.h"

void VolumeClassifier(objectMessage &object)
{
	//If we classify something as shooter/scanthecode, leave it alone
	if (object.name == navigator_msgs::PerceptionObject::START_GATE_BUOY) {
		return;
	}
		//THESE VAlUES WILL ALL BE ADJUSTED ON NEXT LAKE DAY!
			//Scan_the_code or shooter
		if (object.maxHeightFromLidar >= 1.25 && object.scale.x > 3 && object.scale.y > 3) {
			object.name = navigator_msgs::PerceptionObject::DETECT_DELIVER_PLATFORM;
		} else if (object.maxHeightFromLidar >= 0.75 && object.scale.x > 2 && object.scale.y > 2) {
			object.name = navigator_msgs::PerceptionObject::SCAN_THE_CODE;
		} else if (object.maxHeightFromLidar >= -0.25 && object.scale.x > 1.5 && object.scale.y > 1.5) {
			object.name = navigator_msgs::PerceptionObject::TOTEM;
		} else if (object.maxHeightFromLidar < -0.25 && object.scale.x <= 1.5 && object.scale.y <= 1.5) {
			object.name = navigator_msgs::PerceptionObject::BUOY;
		} else {
			object.name = navigator_msgs::PerceptionObject::UNKNOWN;
		}

		/*if (object.scale.x > 4 && object.scale.y > 4 && object.scale.z > 1.5 && object.maxHeightFromLidar >= 1.3 ) {
			
		} else if (object.scale.x > 1.5 && object.scale.y > 1.5 && object.scale.z > 0.9906 && object.maxHeightFromLidar >= 1.3 ) {
			
		} else if (object.scale.x > 1.5 && object.scale.y > 1.5 && object.scale.z > 0.9906 && object.maxHeightFromLidar >= 0.1 ) {
			
		} else if (object.strikesPersist.size() > 50) {
			object.name = navigator_msgs::PerceptionObject::BUOY;
		}*/
}
