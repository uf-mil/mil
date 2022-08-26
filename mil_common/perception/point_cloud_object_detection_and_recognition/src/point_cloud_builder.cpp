#include <chrono>
#include <point_cloud_object_detection_and_recognition/point_cloud_builder.hpp>

// point_overload* point_cast(pcodar::point_t& pt)
// {
//   return reinterpret_cast<point_overload*>(&pt);
// }

#define point_cast(x) *reinterpret_cast<point_overload*>(&x)

namespace pcodar
{
size_t findPoint(point_t pt, point_cloud_ptr mega_point_cloud)
{
  for (size_t i = 0; i < mega_point_cloud->points.size(); i++)
  {
    if (point_cast(pt) == point_cast(mega_point_cloud->points[i]))
    {
      return i;
    }
  }
  return -1;  // max value indicates failure to find value
}

unsigned int PointCloudCircularBuffer::number_threads_ = std::thread::hardware_concurrency();

PointCloudCircularBuffer::PointCloudCircularBuffer() : mega_cloud_(boost::make_shared<point_cloud>())
{
  if (number_threads_ < 4)
    number_threads_ = 4;

  threads_.reserve(number_threads_);
  completed_threads_ = new std::atomic<bool>[number_threads_];
  reset_atomics();
  counter_ = 0;
  capacity_ = 0;
}

void PointCloudCircularBuffer::add_point_cloud(const point_cloud_ptr& pc)
{
  auto start = std::chrono::high_resolution_clock::now();
  reset_atomics();
  for (int i = 0; i < pc->points.size(); i++)
  {
    auto& pt = pc->points[i];
    points_to_cloud_[point_cast(pt)].push_back(counter_);
    points_list_[counter_].push_back((point_t*)&points_to_cloud_.find(point_cast(pt))->first);
  }
  *mega_cloud_ += *pc;
  counter_++;
  // std::cout << "first " << points_list_[prev_clouds_.front().get()].size() << std::endl;
  if (prev_clouds_.full())
  {
    std::vector<std::reference_wrapper<const point_t>> points_;
    std::vector<size_t> indices_;
    // auto* front = prev_clouds_.front().get();
    size_t front = counter_ - capacity_;
    for (auto* pt : points_list_[front])
    {
      auto& vec = points_to_cloud_[point_cast(*pt)];
      auto it = std::find(vec.begin(), vec.end(), front);
      if (it != vec.end())
      {
        vec.erase(it);
      }

      if (vec.empty())
      {
        points_.push_back(*pt);
        indices_.push_back(points_.size() - 1);
      }
    }
    // std::cout << points_.size() << std::endl;
    handle_old_points_deletion(points_, start);
    for (auto index : indices_)
    {
      points_to_cloud_.erase(point_cast(points_[index]));
    }
    points_list_.erase(front);
  }

  // Add new cloud to buffer
  prev_clouds_.push_back(pc);

  // // p1 p2 p3 p4 p5 -> p6
  // //

  // // Don't construct mega cloud until buffer of recent clouds is full
  // if (!prev_clouds_.full())
  //   return;

  // //  Assemble cloud as union of buffered clouds
  // mega_cloud_->clear();
  // for (const auto& cloud : prev_clouds_)
  // {
  //   *mega_cloud_ += *cloud;
  // }
}

void PointCloudCircularBuffer::handle_thread(const std::vector<std::reference_wrapper<const point_t>>& points_list_,
                                             const std::vector<int>& cumulative_sizes_, std::vector<size_t>& indices_,
                                             int index)
{
  int low = cumulative_sizes_[index];
  int high = (index + 1 == cumulative_sizes_.size()) ? points_list_.size() : cumulative_sizes_[index + 1];
  for (int i = low; i < high; i++)
  {
    size_t index = findPoint(points_list_[i], mega_cloud_);
    indices_[i] = index;
  }
  completed_threads_[index].store(true);
}

void PointCloudCircularBuffer::handle_old_points_deletion(
    const std::vector<std::reference_wrapper<const point_t>>& points_list_,
    std::chrono::time_point<std::chrono::high_resolution_clock>& start)
{
  std::vector<size_t> indices_(points_list_.size());
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  int per_thread_ = points_list_.size() / number_threads_;
  std::vector<int> sizes_(number_threads_, per_thread_);
  std::vector<int> cumulative_sizes_(number_threads_);
  int remainder_ = points_list_.size() % number_threads_;

  if (remainder_ != 0)
  {
    for (int i = 0; i < remainder_; i++)
    {
      sizes_[i]++;
    }
  }

  int count = 0;
  for (int i = 0; i < sizes_.size(); i++)
  {
    cumulative_sizes_[i] = count;
    count += sizes_[i];
  }
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

  for (int i = 0; i < number_threads_; i++)
  {
    threads_.push_back(std::thread(&PointCloudCircularBuffer::handle_thread, this, std::cref(points_list_),
                                   std::cref(cumulative_sizes_), std::ref(indices_), i));
  }

  while (true)
  {
    int count = 0;
    for (int i = 0; i < number_threads_; i++)
    {
      if (completed_threads_[i].load())
      {
        count++;
      }
    }
    if (count == number_threads_)
      break;
  }

  std::cout << duration.count() << std::endl;

  std::sort(indices_.begin(), indices_.end());

  for (auto it = indices_.begin(); it != indices_.end(); it++)
  {
    inliers->indices.push_back(*it);
  }
  extract.setInputCloud(mega_cloud_);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*mega_cloud_);
}

void PointCloudCircularBuffer::reset_atomics()
{
  for (int i = 0; i < number_threads_; i++)
  {
    completed_threads_[i].store(false);
  }
}

void PointCloudCircularBuffer::clear()
{
  mega_cloud_->clear();
  prev_clouds_.clear();
}

void PointCloudCircularBuffer::update_config(Config const& config)
{
  prev_clouds_.set_capacity(config.accumulator_number_persistant_clouds);
  capacity_ = config.accumulator_number_persistant_clouds;
}

point_cloud_ptr PointCloudCircularBuffer::get_point_cloud()
{
  return mega_cloud_;
}

}  // namespace pcodar
