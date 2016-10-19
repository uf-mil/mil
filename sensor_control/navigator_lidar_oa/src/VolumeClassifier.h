#include "ConnectedComponents.h"

void VolumeClassifier(objectMessage &object)
{
	//If we classify something as shooter/scanthecode, leave it alone
	if (object.name == "shooter" || object.name == "scanthecode" || object.name == "tower") {
		return;
	}

	if (object.scale.x > 4 && object.scale.y > 4 && object.scale.z > 2 && object.strikesPersist.size() > 600 ) {
		object.name = "shooter";
	} else if (object.scale.x > 1.5 && object.scale.y > 1.5 && object.scale.z > 1.5 && object.strikesPersist.size() > 300) {
		object.name = "tower";
	} else if (false) {
		object.name = "scanthecode";
	} else if (object.strikesPersist.size() > 50) {
		object.name = "buoy";
	}

}