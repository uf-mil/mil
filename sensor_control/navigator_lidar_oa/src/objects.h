#pragma once
#include <vector>
#include <iostream>
#include "ConnectedComponents.h"

/*
	This gives you a list of objects, where every objects that is classified as 'the same' is given a persistant id.
	This list does not persist past what is seen in immediate view
*/

class ObjectTracker{

private:
	std::vector<objectMessage> saved_objects;
	float diff_thresh;
	int curr_id = 0;

public:

	ObjectTracker(float diff_thresh=4){
		this->diff_thresh = diff_thresh;

	}

	std::vector<objectMessage> add_objects(std::vector<objectMessage> objects){
		std::vector<objectMessage> new_objects;
		for(auto obj : objects){
			float min_dist = 100;
			objectMessage min_obj;
			for(auto s_obj : saved_objects){
				float xdiff = pow(obj.position.x - s_obj.position.x, 2);
				float ydiff = pow(obj.position.y - s_obj.position.y, 2);
				float zdiff = pow(obj.position.z - s_obj.position.z, 2);
				float diff = sqrt(xdiff+ydiff+zdiff);
				// TODO MAKE THIS A ROS PARAM
				if(diff < min_dist){
					min_dist = diff;
					min_obj = s_obj;
				}
			}
			if(min_dist < diff_thresh){
				auto a = objectMessage();
				a.scale = obj.scale;
				a.position = obj.position;
				a.id = min_obj.id;
				a.beams = obj.beams;
				new_objects.push_back(a);

			}else{
				obj.id = curr_id;
				++curr_id;
				new_objects.push_back(obj);
				saved_objects.push_back(obj);
			}
		}
		return new_objects;
	}
};
