#pragma once
#include <vector>
#include <iostream>
#include "ConnectedComponents.h"
#include "FitPlanesToCloud.h"
#include "VolumeClassifier.h"
/*
	This gives you a list of objects, where every objects that is classified as 'the same' is given a persistant id.
	This list does not persist past what is seen in immediate view
*/

class ObjectTracker{

private:
	
	float diff_thresh;
	int curr_id = 0;

public:

	ObjectTracker(float diff_thresh=1.5){
		this->diff_thresh = diff_thresh;
	}

	std::vector<objectMessage> add_objects(std::vector<objectMessage> objects, sensor_msgs::PointCloud &rosCloud, const geometry_msgs::Pose &boatPose_enu)
	{
		for(auto obj : objects) {
			float min_dist = 100;
			objectMessage *min_obj;
			for(auto &s_obj : saved_objects){
				float xdiff = pow(obj.position.x - s_obj.position.x, 2);
				float ydiff = pow(obj.position.y - s_obj.position.y, 2);
				float zdiff = pow(obj.position.z - s_obj.position.z, 2);
				float diff = sqrt(xdiff+ydiff+zdiff);
				// TODO MAKE THIS A ROS PARAM
				if(diff < min_dist){
					min_dist = diff;
					min_obj = &s_obj;
				}
			}
			if(min_dist < diff_thresh) {
				obj.name = min_obj->name;
				obj.id = min_obj->id;
				obj.normal = min_obj->normal;
				obj.pclInliers = min_obj->pclInliers;
				*min_obj = obj;
			}else{
				obj.id = curr_id;
				++curr_id;
				saved_objects.push_back(obj);
			}
		}

		for(auto &s_obj : saved_objects) {
			FitPlanesToCloud(s_obj,rosCloud,boatPose_enu);
			VolumeClassifier(s_obj);
		}

		return saved_objects;
	}

	std::vector<objectMessage> saved_objects;
};
