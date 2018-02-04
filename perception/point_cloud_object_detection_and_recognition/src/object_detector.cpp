#include <point_cloud_object_detection_and_recognition/object_detector.hpp>
#include <point_cloud_object_detection_and_recognition/point_cloud_clusterer.hpp>

namespace pcodar
{

    mil_msgs::PerceptionObjectArray object_detector::get_objects(ros::Publisher &pub_pcl_)
    {
        auto accrued_cloud = pc_builder_.get_point_cloud();
        // std::cout << accrued_cloud.size() << std::endl;
        // accrued_cloud.header.frame_id = "velodyne";
        // pub_pcl_.publish(accrued_cloud);
        const auto objects = get_point_cloud_clusters(accrued_cloud);
        //std::cout << objects.size() << std::endl;
        mil_msgs::PerceptionObjectArray object_array;
        object_array.objects = objects;
        return object_array;

    }
    void object_detector::add_point_cloud(const sensor_msgs::PointCloud2& pcloud2, const Eigen::Affine3d& e_velodyne_to_enu)
    {
        pc_builder_.add_point_cloud(pcloud2, e_velodyne_to_enu);
    }

} // namespace pcodar
