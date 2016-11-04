#pragma once
#include <vector>
#include <iostream>
#include "ConnectedComponents.h"
#include "FitPlanesToCloud.h"
#include "VolumeClassifier.h"
#include <boost/assert.hpp>
#include <tuple>
#include <Eigen/Dense>
#include <ros/console.h>
/*
	This gives you a list of objects, where every objects that is classified as 'the same' is given a persistant id.
	This list does not persist past what is seen in immediate view
*/

class ObjectTracker{

private:
	
	float diff_thresh;
	int curr_id = 0;
	bool foundGates = false;
	std::vector<geometry_msgs::Point> gatePositions;

public:

	ObjectTracker(float diff_thresh=4){
		this->diff_thresh = diff_thresh;
	}

	std::vector<objectMessage> add_objects(std::vector<objectMessage> objects, sensor_msgs::PointCloud &rosCloud, const geometry_msgs::Pose &boatPose_enu)
	{
		auto cnt = 0;
		for (auto &ii : objects) {
			++cnt;
			for (auto jj = objects.begin()+cnt; jj != objects.end(); ++jj) {
				auto xyDistance = sqrt( pow(ii.position.x - jj->position.x, 2) + pow(ii.position.y - jj->position.y, 2)  );
				std::stringstream ss;
				ss << "Raw object position failure: " << ii.position.x << "," << ii.position.y << " vs " << jj->position.x << "," << jj->position.y;
				BOOST_ASSERT_MSG(xyDistance > diff_thresh/3, ss.str().c_str());
			}
		}

		for(auto &s_obj : saved_objects) {
			s_obj.current = false;
		}


		for(auto &obj : objects) {
			if (!obj.real) {continue;}
			float min_dist = diff_thresh;
			objectMessage *min_obj;
			for(auto &s_obj : saved_objects){
				auto xyDistance = sqrt( pow(obj.position.x - s_obj.position.x, 2) + pow(obj.position.y - s_obj.position.y, 2) );
				auto scaleDiff = sqrt( pow(obj.scale.x - s_obj.scale.x, 2) + pow(obj.scale.y - s_obj.scale.y, 2) + pow(obj.scale.z - s_obj.scale.z, 2) );
				if(xyDistance < min_dist){
					min_dist = xyDistance;
					min_obj = &s_obj;
				}
			}
			if(min_dist < diff_thresh) {
				obj.name = min_obj->name;
				obj.id = min_obj->id;
				obj.normal = min_obj->normal;
				obj.pclInliers = min_obj->pclInliers;
				obj.color = min_obj->color;
				obj.current = true;
				obj.locked = min_obj->locked;
				obj.real = min_obj->real;
				*min_obj = obj;
			}else{
				obj.id = curr_id++;
				obj.current = true;
				saved_objects.push_back(obj);
			}
		}

		cnt = 0;
		for (auto &ii : saved_objects) {
			++cnt;
			if (!ii.real) {continue;}
			for (auto jj = saved_objects.begin()+cnt; jj != saved_objects.end(); ++jj) {
				auto xyDistance = sqrt( pow(ii.position.x - jj->position.x, 2) + pow(ii.position.y - jj->position.y, 2)  );
				std::stringstream ss;
				ss << "Database object position failure: " << ii.position.x << "," << ii.position.y << " vs " << jj->position.x << "," << jj->position.y;
				BOOST_ASSERT_MSG(xyDistance > diff_thresh, ss.str().c_str());
			}
		}

		for(auto &s_obj : saved_objects) {
			FitPlanesToCloud(s_obj,rosCloud,boatPose_enu);
			VolumeClassifier(s_obj);
		}

		return saved_objects;
	}

	void addROI(std::string name) {
		objectMessage obj;
		obj.name = name;
		obj.real = false;
		obj.id = curr_id++;
		saved_objects.push_back(obj);
	}

	void lock(std::string name) {
		for(auto &s_obj : saved_objects) {
			if (s_obj.name == name) {
				s_obj.locked = true;
			}
		}		
	}

	bool lookUpByName(std::string name, std::vector< navigator_msgs::PerceptionObject > &objects) {
		for (const auto &obj : saved_objects) {
			if (name == "all" || name == obj.name) {
				if (!obj.real && !obj.locked) { continue; }
				navigator_msgs::PerceptionObject thisOne;
				thisOne.name = obj.name;
				thisOne.position = obj.position;
				thisOne.id = obj.id;
				thisOne.confidence = 0;
				thisOne.size.z = obj.scale.z;
				thisOne.size.x = obj.scale.x;
				thisOne.size.y = obj.scale.y;
				thisOne.points = obj.strikesFrame;
				thisOne.intensity = obj.intensityFrame;
				thisOne.pclInliers = obj.pclInliers;
				thisOne.normal = obj.normal;
				thisOne.color = obj.color;
				objects.push_back(thisOne);
			}
		}
        return objects.size() > 0;
	}

	std::vector<geometry_msgs::Point> FindThreeGates()
	{

		if (foundGates) { return gatePositions; }

		std::vector< int > ids;
		for (auto &obj : saved_objects) {
			if (obj.name == navigator_msgs::PerceptionObject::TOTEM) {
				ids.push_back(obj.id);
			}
		}
		auto combos = combinations(ids.size());

		
		for (auto &permute: combos) {
			geometry_msgs::Point center;
			for (auto id : permute) {
				center.x += saved_objects[ids[id]].position.x/4.;
				center.y += saved_objects[ids[id]].position.y/4.;
				center.z += 0; //saved_objects[ids[id]].position.z/4.;
			}
			gatePositions.push_back(center);
			std::vector< std::tuple<double,int> > order;
			for (auto id : permute) {
				auto distance = sqrt( pow(saved_objects[ids[id]].position.x-center.x,2) + pow(saved_objects[ids[id]].position.y-center.y,2) );
				ROS_INFO_STREAM("LIDAR | Rank: " << distance << "," << ids[id]);
				order.push_back( std::make_tuple(distance,ids[id]) );
			}			
			std::sort(order.begin(),order.end());
			auto edge1 = std::get<1>(order[2]);
			auto edge2 = std::get<1>(order[3]);
			auto distance = sqrt( pow(saved_objects[edge1].position.x-saved_objects[edge2].position.x,2) + pow(saved_objects[edge1].position.y-saved_objects[edge2].position.y,2) );
			auto m = (saved_objects[edge1].position.y-saved_objects[edge2].position.y)/(saved_objects[edge1].position.x-saved_objects[edge2].position.x);
			auto b = saved_objects[edge1].position.y - m*saved_objects[edge1].position.x;
			auto error = 0.;
			for (auto id : permute) {
				error += fabs(saved_objects[ids[id]].position.y - m*saved_objects[ids[id]].position.x -b);
			}
			ROS_INFO_STREAM("LIDAR | FOUND 4 TOTEMS: " << ids[permute[0]] << "," << ids[permute[1]] << "," << ids[permute[2]] << "," << ids[permute[3]] << " with edge distance of " << distance << " between " << edge1 << " and " << edge2 << " and line error of ");
			if (distance >= 22 && distance <= 38 && error < 1) {
				Eigen::Vector2d dir1(center.x - saved_objects[edge1].position.x,center.y - saved_objects[edge1].position.y);
				//dir1.normalize();
				geometry_msgs::Point p;
				p.x = saved_objects[edge1].position.x + dir1(0)/3.0;
				p.y = saved_objects[edge1].position.y + dir1(1)/3.0;
				gatePositions.push_back(p);
				Eigen::Vector2d dir2(center.x - saved_objects[edge2].position.x,center.y - saved_objects[edge2].position.y);
				//dir2.normalize();
				p.x = saved_objects[edge2].position.x + dir2(0)/3.0;
				p.y = saved_objects[edge2].position.y + dir2(1)/3.0;
				gatePositions.push_back(p);
				foundGates = true;
				return gatePositions;
			}
		}
		return gatePositions;
	}

	std::vector< std::vector<int> > combinations(int n, int r = 4)
	{
		std::vector< std::vector<int> > combos;
		if ( n < r) { return combos; }
		std::string bits(r,1);
		bits.resize(n,0);
		do {
			std::vector<int> permute;
			for (auto ii = 0; ii < n; ++ii) {
				if (bits[ii]) { permute.push_back(ii); }
			}
			combos.push_back(permute);
		} while (std::prev_permutation(bits.begin(),bits.end()));
		return combos;
	}

	std::vector<objectMessage> saved_objects;
};
