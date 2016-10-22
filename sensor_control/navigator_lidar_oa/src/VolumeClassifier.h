#include "ConnectedComponents.h"
#include "navigator_msgs/PerceptionObject.h"

void VolumeClassifier(objectMessage &object)
{
	//If we classify something as shooter/scanthecode, leave it alone
	if (object.name == navigator_msgs::PerceptionObject::START_GATE_BUOY || object.name == navigator_msgs::PerceptionObject::SCAN_THE_CODE) {
		return;
	}
		//THESE VAlUES WILL ALL BE ADJUSTED ON NEXT LAKE DAY!
		if (object.scale.x > 4 && object.scale.y > 4 && object.scale.z > 2 && object.strikesPersist.size() > 600 ) {
			object.name = navigator_msgs::PerceptionObject::DETECT_DELIVER_PLATFORM;
		} else if (object.scale.x > 1.5 && object.scale.y > 1.5 && object.scale.z > 1.5 && object.strikesPersist.size() > 300) {
			object.name = navigator_msgs::PerceptionObject::TOTEM;
		} else if (false) {
			object.name = navigator_msgs::PerceptionObject::SCAN_THE_CODE;
		} else if (object.strikesPersist.size() > 50) {
			object.name = navigator_msgs::PerceptionObject::BUOY;
		}
}