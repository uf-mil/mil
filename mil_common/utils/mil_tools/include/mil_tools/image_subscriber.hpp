#ifndef IMAGE_SUBSCRIBER_H_
#define IMAGE_SUBSCRIBER_H_

#include <type_traits>
#include <functional>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <boost/optional.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>

template <typename T = void>
class ImageSubscriber
{
    public:
        template <typename = typename std::enable_if<std::is_same<void, T>::value == false>>
        ImageSubscriber(const std::string& topic, std::function<void(T*, cv::Mat&)>& func, T* a,
                        const std::string& encoding = "bgr8", int queue_size = 1) : encoding_(encoding),
                                                                                    queue_size_(queue_size),
                                                                                    it_(nh_)
        {
            image_subscriber_ = it_.subscribe(topic, queue_size, &ImageSubscriber::imageCallback, this);
            function_ = std::bind(func, a, std::placeholders::_1);
            camera_info_sub_ = nh_.subscribe(getInfoTopic(topic), queue_size, &ImageSubscriber::info_callback, this);
        }

        template <typename = typename std::enable_if<std::is_same<void, T>::value>>
        ImageSubscriber(const std::string& topic, std::function<void(cv::Mat&)>& func,
                        const std::string& encoding = "bgr8", int queue_size = 1) : encoding_(encoding),
                                                                                    queue_size_(queue_size),
                                                                                    it_(nh_)
        {
            image_subscriber_ = it_.subscribe(topic, queue_size, &ImageSubscriber::imageCallback, this);
            function_ = func;
            camera_info_sub_ = nh_.subscribe(getInfoTopic(topic), queue_size, &ImageSubscriber::info_callback, this);
        }

        std::string getInfoTopic(const std::string& input)
        {
            int location = input.rfind('/');
            if(location == input.length() - 1)
                location = input.rfind('/', input.length() - 2);
            return input.substr(0, location - 1) + "/camera_info";
        }

        boost::optional<sensor_msgs::CameraInfo> waitForCameraInfo(unsigned int timeout = 10)
        {
            ROS_WARN("Blocking -- waiting at most %d seconds for camera info.", timeout);

            auto time = ros::Duration(timeout);
            time.sleep();
            auto start_time = ros::Time::now();

            while(ros::Time::now() - start_time < time && ros::ok())
            {
                if(camera_info_.has_value())
                {
                    ROS_INFO("Camera info found!");
                    return camera_info_.get();
                }
                ros::Duration(0.2).sleep();
            }

            ROS_ERROR("Camera info not found.");
            return boost::none;
        }

        bool waitForCameraModel(image_geometry::PinholeCameraModel& pinhole_camera, unsigned int timeout = 10)
        {
            auto msg = waitForCameraInfo(timeout);
            if(!msg)
                return false;
            pinhole_camera.fromCameraInfo(msg);
            return true;
        }

        void info_callback(const sensor_msgs::CameraInfo& info)
        {
            camera_info_sub_.shutdown();
            camera_info_.emplace(info);
        }

        void imageCallback(const sensor_msgs::ImageConstPtr& image)
        {
            try
            {
                function_(cv_bridge::toCvCopy(image, encoding_)->image);
            }
            catch(cv_bridge::Exception& e)
            {
                ROS_ERROR("Could not convert from '%s' to '%s'.", image->encoding, encoding_);
            }
        }
    private:
        std::string encoding_;
        int queue_size_;

        std::function<void(cv::Mat&)> function_;

        ros::NodeHandle nh_;
        image_transport::ImageTransport it_(nh_);
        image_transport::Subscriber image_subscriber_;

        ros::Subscriber camera_info_sub_;
        boost::optional<sensor_msgs::CameraInfo> camera_info_;
};
#endif // IMAGE_SUBSCRIBER_H_
