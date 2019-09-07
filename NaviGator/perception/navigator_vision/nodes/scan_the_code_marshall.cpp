#include "ros/ros.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cstring>
#include <cmath>
#include <iostream>
#include <fstream>
#include <pcl/filters/extract_indices.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <mil_msgs/ObjectDBQuery.h>

//this is the Lidar Analyzer class. Only one is every created.
#define _USE_MATH_DEFINES
class LidarAnalyzer
{
public:
  tf::TransformListener enuToVelodyneListener;//tf listener for setRvizPointCallback
  geometry_msgs::PointStamped stcCenterEnu;
  ros::ServiceClient service_client_;
  
  //tf::TransformBroadcaster velodyneToVelodyneMarshallBroadcaster; 
  //tf::Transform velodyneToVelodyneMarshallTransform;
  LidarAnalyzer()
  {
    
    service_client_ = n_.serviceClient<mil_msgs::ObjectDBQuery>("/database/requests");

    // TODO: do this only when
    mil_msgs::ObjectDBQuery::Request req;
    req.name = "stc_platform";
    mil_msgs::ObjectDBQuery::Response res;
    if (!service_client_.call(req, res)) throw std::runtime_error("bla2");
    if (res.objects.size() == 0) throw std::runtime_error("res.objects.size() == 0");
    //stcCenterEnu.point = res.objects[0].pose.position;
    for (size_t i=0; i<res.objects.size();++i)
    {
      stcCenterEnu.point.x+=res.objects[i].pose.position.x;

      stcCenterEnu.point.y+=res.objects[i].pose.position.y;

      stcCenterEnu.point.z+=res.objects[i].pose.position.z;
    }
    stcCenterEnu.point.x/=res.objects.size();
    stcCenterEnu.point.y/=res.objects.size();
    stcCenterEnu.point.z/=res.objects.size();
    stcCenterEnu.header = res.objects[0].header; 
    // END TODO block
    

    pub_debug_points = n_.advertise<sensor_msgs::PointCloud2>("stc_led_pts_marshall", 1);
    pub_debug2_points = n_.advertise<sensor_msgs::PointCloud2>("marshall_debug", 1);
    sub_velodyne_points = n_.subscribe("velodyne_points", 1000, &LidarAnalyzer::pointCloudAnalysisCallback, this);

    //sub_rviz_point = n_.subscribe("clicked_point",1000, &LidarAnalyzer::rvizPointSetCallback, this);
   
  }

  void pointCloudAnalysisCallback(const sensor_msgs::PointCloud2::ConstPtr& input)
  {
    
    //if we dont have the point from RVIZ yet, kill the callback and well see if we have one later.
    if (!gotRvizPt)
    {
      ROS_ERROR("no STC center yet, cant do no math:-(, please feed me an rviz point");
      return;
    }

    geometry_msgs::PointStamped stcCenterVelodyne;
    try
    {
      // ENU is global, so time can be anything
      stcCenterEnu.header.stamp = input->header.stamp;
      this->enuToVelodyneListener.waitForTransform("/velodyne", stcCenterEnu.header.frame_id , input->header.stamp, ros::Duration(20.0));

      ROS_INFO("got transform!");
      this->enuToVelodyneListener.transformPoint("/velodyne", stcCenterEnu, stcCenterVelodyne);
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("Received an exception trying to transform a point in pointCloudAnalysisCallback: %s", ex.what());
      return;
    }
    
    stcCenter[0] = stcCenterVelodyne.point.x;
    stcCenter[1] = stcCenterVelodyne.point.y;
    stcCenter[2] = stcCenterVelodyne.point.z;
    ROS_INFO("stcCenter[0]: %.2f", stcCenter[0]);
    ROS_INFO("stcCenter[1]: %.2f", stcCenter[1]);
    ROS_INFO("stcCenter[2]: %.2f", stcCenter[2]);
    
    //initialize pcl_pc2
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input, pcl_pc2);//saves input as pcl_pc2
    //initialize pt_cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr pt_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *pt_cloud);
    this->cloudOfIntrest = pt_cloud; 
    pcl::PointCloud<pcl::PointXYZ>::Ptr ledPts(new pcl::PointCloud<pcl::PointXYZ>);     
    //TODO make service call to pcodar for center of stc_platform in the RF of the velodyne
    ROS_INFO("cloudOfIntrest %i", cloudOfIntrest->size());  
    this->cylinderPrune(this->radius, this->stcCenter, this->cloudOfIntrest); 
    ROS_INFO("cyindar prune finished");
    
    this->cloudRotateZ(this->stcCenter,this->cloudOfIntrest, -1);
    ROS_INFO("rotation finfished");
    
    ledPts = this->calcLedPts();
    ROS_INFO("calcLedPts finished");
    
    //ledPts = cloudOfIntrest; 
    this->cloudRotateZ(this->stcCenter,ledPts, 1);//now we rotate back
    //this->cloudRotateZ(this->stcCenter,cloudRefined, 1);//now we rotate back
    //ROS_INFO("rotation back finfished\n\n");
    sensor_msgs::PointCloud2 output_msg;
    //pcl::toROSMsg(*ledPts, output_msg);
 
    //ROS_INFO("toROSmsg");
    pcl::toROSMsg(*ledPts, output_msg);
    //ROS_INFO("toROSmsg");
    output_msg.header = input->header;
    pub_debug_points.publish(output_msg);

    //ledPts->clear();
    
    //partitions.erase(partitions.begin(),partitions.end());
    /*
    //debug
    this->cloudRotateZ(this->stcCenter,this->cloudOfIntrest, 1); 
    sensor_msgs::PointCloud2 output_msg2;
    pcl::toROSMsg(*cloudRefined, output_msg2); 
    output_msg2.header = input->header; 
    pub_debug2_points.publish(output_msg);
    //end debug
    */
    return;
  }

/*
  void rvizPointSetCallback(const geometry_msgs::PointStamped::ConstPtr& stcCenterENU)//this works half of time
  //errors out the other half in simulation due to
  //Lookup would require extrapolation into the past.
  {  
    ROS_INFO("GOT RVIZ POINT");
    stcCenterEnu = *stcCenterENU;
    gotRvizPt = true;
    return;
  }

*/
private:
  ros::NodeHandle n_;
  ros::Publisher pub_debug_points,pub_debug2_points;
  ros::Subscriber sub_velodyne_points, sub_rviz_point;
  int count;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOfIntrest;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudRefined;
 
  //pcl::PointCloud<pcl::PointXYZ>::Ptr ledPts(new pcl::PointCloud<pcl::PointXYZ>);

  float * betaHat = new float [2];
  float stcCenter[3];
  const float radius = 1;//
  const float ledHeight = .6;//tallness of the LED Pannel
  
  bool gotRvizPt = true;

  //unused : const float ledFrac = .5;//the fraction of the surface as a whole that the led panel covers 
 
 
  //tf::TransformBroadcaster br;
  //tf::Transform transform; 
  
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> partitions;
  //function to erase ponts outside of a specified vertical cylendar
 void cylinderPrune
  (
  float radius,
  float center[3],
  pcl::PointCloud<pcl::PointXYZ>::Ptr input
  )
  {
    float dist = pow(radius,2);
    std::vector<size_t> badPtsVec;
    for (size_t i=0; i< input->size(); ++i)
    {
      pcl::PointXYZ pt =  input->points[i];
      float distTemp = (pow(pt.x-center[0],2))+(pow(pt.y-center[1],2));

      if (distTemp>dist)
      {
        badPtsVec.push_back(i);
      }
    }
    size_t j =0;
    for(size_t i=0; i<badPtsVec.size(); ++i)
    {
      input->points.erase(input->points.begin()+badPtsVec.at(i)-j);
      input->points.resize(input->height*input->width);
      ++j;
    }
    for (size_t i =0; i<cloudOfIntrest->size();++i)
    {
      pcl::PointXYZ pt = cloudOfIntrest->points[i];
      if ((pt.x!=pt.x)||(pt.y!=pt.y)||((pt.z!=pt.z)))
        ROS_ERROR("%i is NAN", i);
    }
  }  
  //rotates Cloud of intrest about the z axis so that stc center lines on the x axis
  //this will make it easier to analyze
  void cloudRotateZ
  (
  float center[3],
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
  int dir
  )
  {
    
    ROS_INFO("stcCenter[0]: %.2f", center[0]);
    ROS_INFO("stcCenter[1]: %.2f", center[1]);
    ROS_INFO("stcCenter[2]: %.2f", center[2]);
    float theta = 0;  
    //for quadrant 1:
    if ((center[0]>0)&&(center[1]>0))
    {
      theta = atan(abs(center[1]/center[0]));
      ROS_INFO("quad1");
    }
    //for quadrant 2:
    else if ((center[0]<0)&&(center[1]>0))
    {
      theta = atan(abs(center[1]/center[0]))+(90);
      ROS_INFO("quad2");
    }
    //for quadrant 3:
    else if ((center[0]<0)&&(center[1]<0))
    {
      theta = atan(abs(center[1]/center[0]))+(180);
      //dir*=-1;
      ROS_INFO("quad3");
    }
    //for quadrant 4:
    else if ((center[0]>0)&&(center[1]<0))
    {
      //theta = atan(abs(center[1]/center[0]))+(3*M_PI/2);
      theta = atan(abs(center[1]/center[0]))+(270);
      ROS_INFO("quad4");
      dir*=-1;
    }
    else
      ROS_INFO("IDIOT");
    //now we iterate through the point cloud and rotate each point around the z axis by -theta radians
    for (size_t i=0;i<cloud->size();++i)
    {
      float phi = atan(cloud->points[i].y/cloud->points[i].x);
      float r = sqrt(pow(cloud->points[i].y,2)+pow(cloud->points[i].x,2));
      phi+=(theta*dir);
      cloud->points[i].x = r*cos(phi);//TODO should the cos sin be other way around
      cloud->points[i].y = r*sin(phi);
    }
  }

   
  pcl::PointCloud<pcl::PointXYZ>::Ptr calcLedPts()
  {
    //first we find the most Z pts of the cloudRefined
    pcl::PointXYZ mostZPt = cloudOfIntrest->points[0];
    //ROS_INFO("spooky scarry: %i", cloudRefined->size());
    for(size_t i = 0;i<cloudOfIntrest->size();++i)
    {
      if (cloudOfIntrest->points[i].z>mostZPt.z)
      {
        mostZPt = cloudOfIntrest->points[i];
      }
      //ROS_INFO("aH %.2f", mostZPt.z);
    }
    ROS_INFO("highest Z point %.2f",mostZPt.z);
    //now we find the top and bottom of the led pannel    
    float led2dTop= mostZPt.z;
    
    float led2dBot= led2dTop-ledHeight;
   
    
    //now we grab all the partitions between
    pcl::PointCloud<pcl::PointXYZ>::Ptr _ledPts (new pcl::PointCloud<pcl::PointXYZ>);
    ROS_INFO("finding points between %.2e, and %.2e",led2dBot,led2dTop);
    pcl::PointCloud<pcl::PointXYZ>::Ptr allLedPts (new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i=0; i<cloudOfIntrest->size();++i)
    {
      if (cloudOfIntrest->points[i].z>=led2dBot && (cloudOfIntrest->points[i].z<led2dTop-.2))
      {
        //ROS_INFO("getting outlinepts from %i",i);
        allLedPts->points.push_back(cloudOfIntrest->points[i]);
      }
    }
    ROS_INFO("%i number of ledPts",_ledPts->size());
    
    //return getOutlinePts(allLedPts);
    
    return getOutlinePts(allLedPts);
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr getOutlinePts(pcl::PointCloud<pcl::PointXYZ>::Ptr input)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr outlinePts(new pcl::PointCloud<pcl::PointXYZ>);
    
    


    pcl::PointXYZ bot =input->points[0];//least z
    pcl::PointXYZ top = input->points[0];//most z
    pcl::PointXYZ left =input->points[0];//most Y
    pcl::PointXYZ right = input->points[0];//least y
    for (size_t i=0; i<input->size();++i)
    {  
        if (input->points[i].z>top.z)
          top = input->points[i];        
        if (input->points[i].z<bot.z)
          bot = input->points[i];

        if (input->points[i].y>left.y)
          left = input->points[i];
        if (input->points[i].y<right.y)
          right = input->points[i];
      
    }
    outlinePts->points.push_back(bot);
    outlinePts->points.push_back(top);
    outlinePts->points.push_back(left);
    outlinePts->points.push_back(right);
    return outlinePts;
    /***stuff for considering coner case***
    //y delta between the left most(mostY) and middle point(leastX)
    float distL = sqrt(pow(mostYPt.x-leastXPt.x,2)+pow(mostYPt.y-leastXPt.y,2)+pow(mostYPt.z-leastXPt.z,2));
    //y delta between the right most(leastY) and middle point(leastX)
    //float distR = abs(leastYPt.y-leastXPt.y);
    float distR = sqrt(pow(leastYPt.x-leastXPt.x,2)+pow(leastYPt.y-leastXPt.y,2)+pow(leastYPt.z-leastXPt.z,2));
    //float distT = abs(leastYPt.y-mostYPt.y); 
    float distT = sqrt(pow(mostYPt.x-leastYPt.x,2)+pow(mostYPt.y-leastYPt.y,2)+pow(mostYPt.z-leastYPt.z,2));
    if (distT<(distL+distR+.05)&&(distT>(distL+distR-.05)))//if not the corner case, approx a line
    {
      outlinePts->points.push_back(mostYPt);
      outlinePts->points.push_back(leastYPt);//left, then right pt
      return outlinePts;
    }
    //if is corner case 
    if (distR>distL)
    {
      outlinePts->points.push_back(leastXPt);//left then right pt
      outlinePts->points.push_back(leastYPt); 
    }
    else if (distL>distR)
    {
      outlinePts->points.push_back(mostYPt);//left then right pt 
      outlinePts->points.push_back(leastXPt);
    }
    
    //outlinePts->points.push_back(mostYPt); 
    //outlinePts->points.push_back(leastYPt); 
    //outlinePts->points.push_back(leastXPt);
    return outlinePts;*/
  }
};//End of class LidarAnalyzer

int main (int argc, char **argv)
{
  ros::init(argc, argv, "scan_the_code_marshall");
  
  LidarAnalyzer instanceOfLidarAnalyzer;
  ros::spin();
  return 0;
}


