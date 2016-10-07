#include <vector>
#include <iostream>
#include <Eigen/Geometry>
#include "ConnectedComponents.h"

class BoundingBox{
public:
	static std::vector<objectMessage> get_accurate_objects(const sensor_msgs::PointCloud2ConstPtr &pcloud, 
														std::vector<objectMessage> objects,
														Eigen::Affine3d T_enu_velodyne){
		std::vector<objectMessage> small_objects;
		for(auto obj : objects){
			float xmin = obj.position.x - (obj.scale.x/2 + .8);
			float xmax = obj.position.x + (obj.scale.x/2 + .8);
			float ymin = obj.position.y - (obj.scale.y/2 + .8);
			float ymax = obj.position.y + (obj.scale.y/2 + .8);
			float zmin = obj.position.z - (obj.scale.z/2 + .8);
			float zmax = obj.position.z + (obj.scale.z/2 + .8);
			float newxmin = 100000;
			float newymin = 100000;
			float newzmin = 100000;
			float newzmax = -100000;
			float newymax = -100000;
			float newxmax = -100000;

			for (auto ii = 0, jj = 0; ii < pcloud->width; ++ii,jj+=pcloud->point_step) {
				floatConverter x,y,z,i;
				for (int kk = 0; kk < 4; ++kk)
				{
					x.data[kk] = pcloud->data[jj+kk];
					y.data[kk] = pcloud->data[jj+4+kk];
					z.data[kk] = pcloud->data[jj+8+kk];
					i.data[kk] = pcloud->data[jj+16+kk];
				}
				Eigen::Vector3d xyz_in_velodyne(x.f,y.f,z.f);
				Eigen::Vector3d xyz_in_enu = T_enu_velodyne*xyz_in_velodyne;
				float x_ = xyz_in_enu(0);
				float y_ = xyz_in_enu(1);
				float z_ = xyz_in_enu(2);

				if(x_ > xmin && x_ < xmax && y_ > ymin && y_ < ymax && z_ > zmin && z_ < zmax){
					newxmin = x_ < newxmin ? x_ : newxmin;
					newymin = y_ < newymin ? y_ : newymin;
					newzmin = z_ < newzmin ? z_ : newzmin;
					newxmax = x_ > newxmax ? x_ : newxmax;
					newymax = y_ > newymax ? y_ : newymax;
					newzmax = z_ > newzmax ? z_ : newzmax;
				}			
			}

			newxmin = newxmin < newxmax ? newxmin : newxmax;
			newymin = newymin < newymax ? newymin : newymax;
			newzmin = newzmin < newzmax ? newzmin : newzmax;
			newxmax = newxmin < newxmax ? newxmax : newxmin;
			newymax = newymin < newymax ? newymax : newymin;
			newzmax = newzmin < newzmax ? newzmax : newzmin;


			if(newxmin != 100000 && newymin != 100000 && newzmin !=100000
				&& newxmax != -100000 && newymax != -100000 && newzmax != -100000){
				auto sm_ob = objectMessage();
				sm_ob.position.x = (newxmax + newxmin)/2;
				sm_ob.scale.x = fabs(newxmax - newxmin);
				sm_ob.position.y = (newymax + newymin)/2;
				sm_ob.scale.y = fabs(newymax - newymin);	
				sm_ob.position.z = (newzmax + newzmin)/2;
				sm_ob.scale.z = fabs(newzmax - newzmin);
				small_objects.push_back(sm_ob);	

			}		
		}
		return small_objects;
	}
};