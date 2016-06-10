#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "navigator_msgs/Point.h"
#include "navigator_msgs/Points.h"
#include "pcl/ModelCoefficients.h"
#include "pcl/io/pcd_io.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/features/normal_3d.h"
#include "pcl/kdtree/kdtree.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/common/common.h"
#include <math.h>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/Point.h"

ros::Publisher results;
ros::Publisher visualization;

void scanCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{

    pcl::PCLPointCloud2::Ptr pcl_cloud (new pcl::PCLPointCloud2 ());

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl_conversions::toPCL(*msg, *pcl_cloud);
    pcl::fromPCLPointCloud2(*pcl_cloud, *cloud);

    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud (cloud);
    vg.setLeafSize (0.1f, 0.1f, 0.1f);
    vg.filter (*cloud_filtered);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (.5); // 10cm
    ec.setMinClusterSize (2);
    ec.setMaxClusterSize (20);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);

    navigator_msgs::Points points;
    visualization_msgs::MarkerArray marker_array;
    points.count = 0;


    for (std::vector<pcl::PointIndices>::const_iterator i = cluster_indices.begin (); i != cluster_indices.end (); ++i)
    {

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);

        for (std::vector<int>::const_iterator j = i->indices.begin (); j != i->indices.end (); ++j)
        {
            cloud_cluster->points.push_back (cloud_filtered->points[*j]);
        }


        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = false;

        navigator_msgs::Point point;
        pcl::PointXYZ min_pt;
        pcl::PointXYZ max_pt;

        pcl::getMinMax3D(*cloud_cluster, min_pt, max_pt);
        point.x = (min_pt.x + max_pt.x)/2; 
        point.y = (min_pt.y + max_pt.y)/2; 
        point.radius = 3 * (((max_pt.x - min_pt.x) + (max_pt.y - min_pt.y))/2); 

        if (abs(point.x) < 20 && abs(point.y) < 20)
        {
            points.count++;
            points.points.push_back(point);

            visualization_msgs::Marker marker;
            marker.header.frame_id = "/velodyne";
            marker.ns = "basic_shapes";
            marker.id = points.count;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.lifetime = ros::Duration();
            marker.pose.position.x = point.x;
            marker.pose.position.y = point.y;
            marker.pose.position.z = 0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = point.radius;
            marker.scale.y = point.radius;
            marker.scale.z = 0;
            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;

        marker_array.markers.push_back(marker);
        }
        
    }

    results.publish(points);
    visualization.publish(marker_array);

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "navigator_lidar");

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/velodyne_points", 1000, scanCallback);
  results = n.advertise<navigator_msgs::Points>("/obstacles", 1000);
  visualization = n.advertise<visualization_msgs::MarkerArray>("/obstacle_visualization", 1000);
  ros::spin();

  return 0;

}
