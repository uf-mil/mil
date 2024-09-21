#include <mil_msgs/PerceptionObject.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_trimmed.h>
#include <pcl/search/octree.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <point_cloud_object_detection_and_recognition/object_associator.hpp>
#include <unordered_set>

namespace pcodar
{
void Associator::associate(ObjectMap& prev_objects, point_cloud const& pc, clusters_t clusters, bool moving_back)
{
  // Tracks which clusters have been seen
  std::unordered_set<uint> seen;

  // Establish Area of Interest (AOI) that new points are not allowed to be published while Navigator is moving
  // backwards
  Eigen::Vector3f min_aoi(0.0f, -5.0f, -0.5f);  // Min bounds (x, y, z)
  Eigen::Vector3f max_aoi(10.0f, 5.0f, 0.5f);   // Max bounds (x, y, z)

  // Iterate through each new cluster, finding which persistent cluster(s) it matches
  for (cluster_t const& cluster : clusters)
  {
    // Make pointcloud from this pointcloud
    point_cloud_ptr cluster_pc = boost::make_shared<point_cloud>(pc, cluster.indices);
    KdTreePtr cluster_search_tree = boost::make_shared<KdTree>();
    cluster_search_tree->setInputCloud(cluster_pc);

    using CorrespondenceEstimation = pcl::registration::CorrespondenceEstimation<point_t, point_t>;
    CorrespondenceEstimation ce;
    ce.setSearchMethodTarget(cluster_search_tree, true);
    ce.setInputTarget(cluster_pc);

    using ObjectMapIt = decltype(prev_objects.objects_.begin());
    std::vector<ObjectMapIt> matches;
    for (auto pair = prev_objects.objects_.begin(); pair != prev_objects.objects_.end(); ++pair)
    {
      ce.setSearchMethodSource((*pair).second.get_search_tree(), true);
      ce.setInputSource((*pair).second.get_points_ptr());

      pcl::CorrespondencesPtr correspondences = boost::make_shared<pcl::Correspondences>();
      ce.determineCorrespondences(*correspondences, max_distance_);

      if (correspondences->size() > 0)
        matches.push_back(pair);
    }

    // If Navigator is moving back, new objects outside of the AOI are invalid
    if (moving_back)
    {
      // Compute the centroid of the cluster
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*cluster_pc, centroid);

      // Transform centroid from /enu to /wamv/baselink
      tf::TransformListener listener;
      tf::StampedTransform transform;

      try
      {
        // Listen for the transform from /enu to /wamv/base_link
        listener.waitForTransform("/wamv/base_link", "/enu", ros::Time(0), ros::Duration(1.0));
        listener.lookupTransform("/wamv/base_link", "/enu", ros::Time(0), transform);

        // Apply the transformation to the centroid
        tf::Vector3 centroid_enu(centroid[0], centroid[1], centroid[2]);
        tf::Vector3 centroid_baselink = transform * centroid_enu;

        // Update centroid with the transformed values
        centroid[0] = centroid_baselink.x();
        centroid[1] = centroid_baselink.y();
        centroid[2] = centroid_baselink.z();

        // Check if the centroid is outside the AOI
        bool outside_aoi = (centroid[0] < min_aoi[0] || centroid[0] > max_aoi[0] ||  // X bounds
                            centroid[1] < min_aoi[1] || centroid[1] > max_aoi[1] ||  // Y bounds
                            centroid[2] < min_aoi[2] || centroid[2] > max_aoi[2]);   // Z bounds

        if (!outside_aoi)
        {
          std::cout << centroid[0] << ", " << centroid[1] << ", " << centroid[2] << "\n";
          continue;
        }
        else
        {
          if (matches.size() == 0)
          {
            // Add to object
            auto id = prev_objects.add_object(cluster_pc, cluster_search_tree);
            seen.insert(id);
          }
          else
          {
            seen.insert((*matches.at(0)).first);
            (*matches.at(0)).second.update_points(cluster_pc, cluster_search_tree);
            for (size_t i = 1; i < matches.size(); ++i)
            {
              prev_objects.erase_object(matches.at(i));
            }
          }
        }
      }
      catch (tf::TransformException& ex)
      {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
        continue;  // Skip this iteration if the transform fails
      }
    }
    else
    {
      if (matches.size() == 0)
      {
        // Add to object
        auto id = prev_objects.add_object(cluster_pc, cluster_search_tree);
        seen.insert(id);
      }
      else
      {
        seen.insert((*matches.at(0)).first);
        (*matches.at(0)).second.update_points(cluster_pc, cluster_search_tree);
        for (size_t i = 1; i < matches.size(); ++i)
        {
          prev_objects.erase_object(matches.at(i));
        }
      }
    }
  }

  // forget any objects that we didn't see, if that functionality is enabled
  if (forget_unseen_)
  {
    for (auto pair = prev_objects.objects_.begin(); pair != prev_objects.objects_.end();)
    {
      if (seen.find((*pair).first) == seen.end())
      {
        pair = prev_objects.erase_object(pair);
      }
      else
      {
        ++pair;
      }
    }
  }
}

void Associator::update_config(Config const& config)
{
  max_distance_ = config.associator_max_distance;
  forget_unseen_ = config.associator_forget_unseen;
}

}  // namespace pcodar
