#include <point_cloud_object_detection_and_recognition/marker_manager.hh>
#include <point_cloud_object_detection_and_recognition/pcodar_params.hh>
#include <point_cloud_object_detection_and_recognition/point_cloud_builder.hh>
#include <point_cloud_object_detection_and_recognition/point_cloud_clusterer.hh>
#include <point_cloud_object_detection_and_recognition/pcodar_controller.hh>

#include <eigen_conversions/eigen_msg.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <geometry_msgs/PointStamped.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <mil_msgs/PerceptionObjectArray.h>

#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2_ros/transform_listener.h>

#include <nav_msgs/Odometry.h>

#include <chrono>
#include <iostream>
#include <thread>

#include <pcl_ros/point_cloud.h>

#include <QApplication>
#include <QDesktopWidget>
#include <QStyle>

#include "label_model.hh"
#include "main_window.hh"

/*
UI Features I want:
- Loading mega cloud percentage in CLI

Buttons I want:
- Publish mega point cloud
- Restart bag
- Right panel w/ list of objects with show check box and drop downs for classification
- Generate bag button
*/

int main(int argc, char *argv[])
{
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    id_to_labeled_object object_map;
    auto object_map_ptr = std::make_shared<id_to_labeled_object>(object_map);

    // TODO(tbianchi) argument checking
    label_model l_model(argc, argv, object_map_ptr);
    l_model.populate_map();

    QApplication a(argc, argv);
    main_window w;
    w.set_object_map(object_map_ptr);
    w.populate_list();

    w.setGeometry(QStyle::alignedRect(Qt::LeftToRight, Qt::AlignCenter, w.size(), a.desktop()->availableGeometry()));
    w.show();

    const auto update = [&w, &l_model, &a]() {

        if (w.restart())
        {
            l_model.restart();
        }
        if (w.generate_bag())
        {
            l_model.generate_bag();
        }
        if (l_model.has_next_image())
        {
            cv::Mat cv_image = l_model.get_next_image();
            cvtColor(cv_image, cv_image, CV_BGR2RGB);
            QImage q_image(cv_image.data, cv_image.cols, cv_image.rows, cv_image.step, QImage::Format_RGB888);
            w.update_image(q_image);
            a.processEvents();
        }

    };

    QTimer *timer = new QTimer(&w);
    w.connect(timer, &QTimer::timeout, update);
    timer->start(33.0);

    return a.exec();
}