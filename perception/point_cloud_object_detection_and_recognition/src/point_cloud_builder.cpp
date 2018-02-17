#include <point_cloud_object_detection_and_recognition/point_cloud_builder.hpp>

#include <eigen_conversions/eigen_msg.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/random_sample.h>


#include <pcl_conversions/pcl_conversions.h>

#include <tf2/convert.h>

namespace pcodar
{
namespace
{
point_cloud filter(const point_cloud& in_cloud, const bool real_time)
{


    point_cloud out_cloud;
    const auto buffered_cloud_ptr = in_cloud.makeShared();
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(buffered_cloud_ptr);
    vg.setLeafSize(.3, .3, .3);
    vg.filter(out_cloud);

    const auto buffered_cloud_ptr_1 = out_cloud.makeShared();
    
    // Initializing with true will allow us to extract the removed indices
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sorfilter(true);
    sorfilter.setInputCloud(buffered_cloud_ptr_1);
    sorfilter.setStddevMulThresh(2.0);
    sorfilter.setMeanK(10);
    sorfilter.filter(out_cloud);

    const auto buffered_cloud_ptr_2 = out_cloud.makeShared();

    int number_points = 100000;
    if(out_cloud.size() < number_points || !real_time)
    {
        return out_cloud;
    }

    pcl::RandomSample<pcl::PointXYZ> sample (true);
    sample.setInputCloud (buffered_cloud_ptr_2);
    sample.setSample (number_points);
    sample.filter(out_cloud);


    return out_cloud;
}
}  // anonymous namespace

point_cloud transform_point_cloud(const sensor_msgs::PointCloud2& pcloud2, const Eigen::Affine3d& e_velodyne_to_X)
{
    // Transform from PCL2
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(pcloud2, pcl_pc2);
    point_cloud pcloud;
    pcl::fromPCLPointCloud2(pcl_pc2, pcloud);

    // Transform points into ENU frame
    point_cloud temp_cloud;
    pcl::transformPointCloud(pcloud, temp_cloud, e_velodyne_to_X);
    return temp_cloud;
}

void point_cloud_builder::add_point_cloud(const sensor_msgs::PointCloud2& pcloud2,
                                          const Eigen::Affine3d& e_velodyne_to_enu)
{
   
    auto transformed_cloud = transform_point_cloud(pcloud2, e_velodyne_to_enu).makeShared();
    point_cloud buffered_cloud;
    if (prev_clouds_.full())
    {
        for (const auto& cloud : prev_clouds_)
        {
            buffered_cloud += *cloud;
        }
        prev_clouds_.clear();
    }
    else
    {
        prev_clouds_.push_back(transformed_cloud);
        return;
    }

    const auto filtered_buffered_cloud = filter(buffered_cloud,real_time_);
    mega_cloud_ += filtered_buffered_cloud;
    real_time_ = true;
    mega_cloud_ = filter(mega_cloud_, real_time_);
}

point_cloud point_cloud_builder::get_point_cloud()
{
    // Returning
    return mega_cloud_;
}

}  // pcodar namespace
