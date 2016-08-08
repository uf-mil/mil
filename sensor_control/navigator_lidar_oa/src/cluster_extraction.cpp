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
#include "pcl/filters/passthrough.h"
#include <math.h>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/Point.h"



class ClusterExtraction {

public:
    ClusterExtraction();
    void lidar_cb(const sensor_msgs::PointCloud2::ConstPtr& msg);

private:
    ros::NodeHandle n;
    pcl::PCLPointCloud2::Ptr pcl_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr converted_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr concatenated_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr inside_filter_x;
    pcl::PointCloud<pcl::PointXYZ>::Ptr inside_filter_y;
    ros::Subscriber sub;;
    ros::Publisher results;
    ros::Publisher visualization;
    ros::Publisher temp;
};


ClusterExtraction::ClusterExtraction()
    :
    n("~"),
    pcl_cloud (new pcl::PCLPointCloud2 ()),
    converted_cloud (new pcl::PointCloud<pcl::PointXYZ>),
    concatenated_cloud (new pcl::PointCloud<pcl::PointXYZ>),
    inside_filter_x (new pcl::PointCloud<pcl::PointXYZ>),
    inside_filter_y (new pcl::PointCloud<pcl::PointXYZ>),
    sub(n.subscribe("/velodyne_points", 1000, &ClusterExtraction::lidar_cb, this)),
    results(n.advertise<navigator_msgs::Points>("/obstacles", 1000)),
    visualization(n.advertise<visualization_msgs::MarkerArray>("/obstacle_visualization", 1000)),
    temp(n.advertise<sensor_msgs::PointCloud2>("/temp", 1000))
    {
        ros::spin();
}

void ClusterExtraction::lidar_cb(const sensor_msgs::PointCloud2::ConstPtr& msg)
{

    pcl::PassThrough<pcl::PointXYZ> pass;
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

    // Convert from sensor_msgs::PointCloud2 -> pcl::PointCloud2 -> pcl::PointCLoud<PointXYZ>
    // The final version is needed for the extraction
    pcl_conversions::toPCL(*msg, *pcl_cloud);
    pcl::fromPCLPointCloud2(*pcl_cloud, *converted_cloud);

    // This series of filters cleans up the cloud for clustering

    // Remore any points futher than 15 meters in the X direction
    pass.setInputCloud (converted_cloud);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (-15, 15.0);
    pass.filter (*converted_cloud);

    // Remore any points futher than 15 meters in the Y direction
    pass.setInputCloud (converted_cloud);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-15, 15);
    pass.filter (*converted_cloud);

    // Take the 30x30 filtered cloud and remove the middle 5 meters in the X axis
    pass.setInputCloud (converted_cloud);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (-2.5, 2.5);
    pass.setFilterLimitsNegative (true);
    pass.filter (*inside_filter_x);

    // Take the 30x30 filtered cloud and remove the middle 5 meters in the Y axis
    pass.setInputCloud (converted_cloud);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-1.5, 1.5);
    pass.filter (*inside_filter_y);

    // Combine these clouds to get a 30x30 cloud with the inside 5 meters removed
    *concatenated_cloud = *inside_filter_x + *inside_filter_y;

    tree->setInputCloud (concatenated_cloud);

    // Given the search tree of the cloud and the cloud, extract the clusters from the cloud

    ec.setClusterTolerance (.1);
    ec.setMinClusterSize (.01);
    ec.setMaxClusterSize (20);
    ec.setSearchMethod (tree);
    ec.setInputCloud (concatenated_cloud);
    ec.extract (cluster_indices);

    // Create variables to hold clusters for publishing and visualizing
    navigator_msgs::Points points;
    visualization_msgs::MarkerArray marker_array;
    points.count = 1;

    // For each cluster in the cloud...
    for (std::vector<pcl::PointIndices>::const_iterator i = cluster_indices.begin (); i != cluster_indices.end (); ++i)
    {

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);

        // Push all the points in the cloud into an array
        // This is data that is needed and used below for visualization
        for (std::vector<int>::const_iterator j = i->indices.begin (); j != i->indices.end (); ++j)
        {
            cloud_cluster->points.push_back (concatenated_cloud->points[*j]);
        }

        // Refactor cloud now that it has points
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = false;

        navigator_msgs::Point point;
        pcl::PointXYZ min_pt;
        pcl::PointXYZ max_pt;

        // Find the Min and Max points of the cluster in the X,Y frame
        pcl::getMinMax3D(*cloud_cluster, min_pt, max_pt);
        point.header.seq = points.count;
        point.header.stamp.sec = ros::Time::now().toSec();
        point.header.stamp.nsec = ros::Time::now().toNSec();
        point.header.frame_id = "velodyne";
        point.x = (min_pt.x + max_pt.x)/2;
        point.y = (min_pt.y + max_pt.y)/2;
        point.radius = 4 * (((max_pt.x - min_pt.x) + (max_pt.y - min_pt.y))/2);

        // If the point is large enough, add it to the points variable and the vis variable
        if (abs(point.x) < 20 && abs(point.y) < 20)
        {
            points.count++;
            // Add to points to points array
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

            // Add marker to marker array
            marker_array.markers.push_back(marker);
        }

    }

    // Fire away

    sensor_msgs::PointCloud2 ros_cloud_out;
    ros_cloud_out.header.frame_id = "velodyne";
    pcl::toROSMsg(*converted_cloud, ros_cloud_out);

    temp.publish(ros_cloud_out);

    results.publish(points);
    visualization.publish(marker_array);

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "navigator_lidar_oa_cluster_extraction");
  ClusterExtraction c;
  return 0;

}


