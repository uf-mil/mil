#include "label_model.hpp"
#include "main_window.cpp"

#include <point_cloud_object_detection_and_recognition/pcodar_params.hpp>
#include <point_cloud_object_detection_and_recognition/point_cloud_builder.hpp>
#include <point_cloud_object_detection_and_recognition/point_cloud_clusterer.hpp>

#include <eigen_conversions/eigen_msg.h>

#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <rosbag/bag.h>

#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2_ros/transform_listener.h>

#include <boost/progress.hpp>

namespace
{
Eigen::Affine3d odom_to_eigen(const nav_msgs::Odometry::ConstPtr odom)
{
    geometry_msgs::Transform g_transform;
    g_transform.translation.x = odom->pose.pose.position.x;
    g_transform.translation.y = odom->pose.pose.position.y;
    g_transform.translation.z = odom->pose.pose.position.z;
    g_transform.rotation = odom->pose.pose.orientation;
    Eigen::Affine3d e_transform;
    tf::transformMsgToEigen(g_transform, e_transform);
    return e_transform;
}

int get_index_of_object(const geometry_msgs::Point32& point, const std::vector<mil_msgs::PerceptionObject>& objects)
{
    for (size_t i = 0; i != objects.size(); ++i)
    {
        const auto& object = objects[i];
        const double yaw = tf::getYaw(object.pose.orientation);
        const Eigen::Rotation2D<double> rot(yaw);
        const Eigen::Vector2d pos = Eigen::Vector2d(object.pose.position.x, object.pose.position.y);

        const double offset_x = object.scale.x / 2.0;
        const double offset_y = object.scale.y / 2.0;

        Eigen::Vector2d a(-offset_x, -offset_y);
        Eigen::Vector2d b(offset_x, -offset_y);
        Eigen::Vector2d d(-offset_x, offset_y);
        Eigen::Vector2d p(point.x, point.y);

        p = p - pos;
        a = (rot * a);
        b = (rot * b);
        d = (rot * d);

        const Eigen::Vector2d ab = b - a;
        const Eigen::Vector2d ad = d - a;
        const Eigen::Vector2d ap = p - a;

        const double ab_ap = ab.dot(ap);
        const double ab_ab = ab.dot(ab);
        const double ad_ap = ad.dot(ap);
        const double ad_ad = ad.dot(ad);

        if (0 < ab_ap && ab_ap < ab_ab && 0 < ad_ap && ad_ap < ad_ad)
        {
            return (int)(i);
        }
    }
    return -1;
}

}  // anonymous namespace



rosbag::View make_view() {}
label_model::label_model(int argc, char* argv[], std::shared_ptr<id_to_labeled_object> object_map_ptr)
    : object_map_ptr_(object_map_ptr)
{
    // Setup ROS
    ros::init(argc, argv, "point_cloud_object_detector");
    ros::Time::init();

    // Set up node handler
    auto nh = ros::NodeHandle(ros::this_node::getName());
    nh_ = std::make_shared<ros::NodeHandle>(nh);

    // Get bag from command-line
    input_bag_name_ = argv[1];
    output_bag_name_ = argv[2];

    bag_.open(input_bag_name_, rosbag::bagmode::Read);

    // This doesn't take into account camera distortion - for this application, it is ok, since we aren't using
    // fisheyes and it's not a performance based application.
    std::vector<std::string> image_topics = {"/odom", "/stereo/right/image_raw"};

    std::vector<std::string> odom_topics = {"/odom"};
    rosbag::View view_odom(bag_, rosbag::TopicQuery(odom_topics));
    last_odom_ = view_odom.begin()->instantiate<nav_msgs::Odometry>();
    v_ = std::make_shared<rosbag::View>(bag_, rosbag::TopicQuery(image_topics));

    bag_iter_ = v_->begin();
}

void label_model::populate_map()
{
    tf2_ros::Buffer tfBuffer(ros::Duration(10.0));
    tf2_ros::TransformListener tf_listener(tfBuffer);
    geometry_msgs::TransformStamped g_velodyne_to_baselink;
    geometry_msgs::TransformStamped g_baselink_to_stereoright;

    ROS_INFO_STREAM("Getting the proper transforms");

    while (true)
    {
        try
        {
            g_velodyne_to_baselink = tfBuffer.lookupTransform("base_link", "velodyne", ros::Time(0));
            g_baselink_to_stereoright = tfBuffer.lookupTransform("stereo_right_cam", "base_link", ros::Time(0));

            break;
        }
        catch (tf2::TransformException& ex)
        {
        }
    }


    //TODO(tbianchi) : Fix this lol
    intrinsics_ << 704, 0, 476, 0, 0, 701, 300, 0, 0, 0, 1, 0;
    tf::transformMsgToEigen(g_velodyne_to_baselink.transform, e_velodyne_to_baselink_);
    tf::transformMsgToEigen(g_baselink_to_stereoright.transform, e_baselink_to_stereoright_);

    ROS_INFO_STREAM("Done getting the proper transforms");

    std::vector<std::string> topics;
    topics.push_back(std::string("/velodyne_points"));
    topics.push_back(std::string("/odom"));

    rosbag::View view(bag_, rosbag::TopicQuery(topics));
    std::vector<std::string> odom_topics = {"/odom"};
    rosbag::View view_odom(bag_, rosbag::TopicQuery(odom_topics));

    nav_msgs::Odometry::ConstPtr last_odom = view_odom.begin()->instantiate<nav_msgs::Odometry>();
    pcodar::point_cloud_builder pc_builder(false);

    ROS_INFO_STREAM("Building mega point cloud");

    const int number_pcloud_messages = view.size() - view_odom.size();

    std::cout << "\nLOADING BAG INTO LABELER" << std::endl;
    
    boost::progress_display show_progress(number_pcloud_messages);

    for (rosbag::MessageInstance& message : view)
    {
        if (message.getTopic() == "/velodyne_points")
        {
            sensor_msgs::PointCloud2::ConstPtr point_cloud = message.instantiate<sensor_msgs::PointCloud2>();

            const Eigen::Affine3d e_baselink_to_enu = odom_to_eigen(last_odom);

            Eigen::Affine3d e_velodyne_to_enu = e_baselink_to_enu * e_velodyne_to_baselink_;

            pc_builder.add_point_cloud(*point_cloud, e_velodyne_to_enu);
            ++show_progress;
        }
        else
        {
            last_odom = message.instantiate<nav_msgs::Odometry>();
        }
    }
    ROS_INFO_STREAM("Done mega building point cloud");

    ROS_INFO_STREAM("Constructing objects");

    mega_cloud_ = pc_builder.get_point_cloud();
    auto objects = pcodar::get_point_cloud_clusters(mega_cloud_);

    std::ranlux48 gen;
    std::uniform_int_distribution<int> uniform_0_255(0, 255);

    uint16_t id_counter = 1;
    for (const auto& o : objects)
    {
        course_object c_object;

        const int r = uniform_0_255(gen);
        const int g = uniform_0_255(gen);
        const int b = uniform_0_255(gen);

        const Eigen::Vector4d position(o.pose.position.x, o.pose.position.y, o.pose.position.z, 1);
        const Eigen::Vector4d p1 = position + Eigen::Vector4d(o.scale.x / 2.0, o.scale.y / 2.0, o.scale.z / 2.0, 0);
        const Eigen::Vector4d p2 = position + Eigen::Vector4d(o.scale.x / 2.0, o.scale.y / 2.0, -o.scale.z / 2.0, 0);
        const Eigen::Vector4d p3 = position + Eigen::Vector4d(o.scale.x / 2.0, -o.scale.y / 2.0, o.scale.z / 2.0, 0);
        const Eigen::Vector4d p4 = position + Eigen::Vector4d(o.scale.x / 2.0, -o.scale.y / 2.0, -o.scale.z / 2.0, 0);
        const Eigen::Vector4d p5 = position + Eigen::Vector4d(-o.scale.x / 2.0, o.scale.y / 2.0, o.scale.z / 2.0, 0);
        const Eigen::Vector4d p6 = position + Eigen::Vector4d(-o.scale.x / 2.0, o.scale.y / 2.0, -o.scale.z / 2.0, 0);
        const Eigen::Vector4d p7 = position + Eigen::Vector4d(-o.scale.x / 2.0, -o.scale.y / 2.0, o.scale.z / 2.0, 0);
        const Eigen::Vector4d p8 = position + Eigen::Vector4d(-o.scale.x / 2.0, -o.scale.y / 2.0, -o.scale.z / 2.0, 0);
        std::vector<Eigen::Vector4d> vec_of_points = {position, p1, p2, p3, p4, p5, p6, p7, p8};

        c_object.p_object = o;
        c_object.p_object.id = id_counter++;
        c_object.p_object.classification = "unknown";
        c_object.bounding_box_points = vec_of_points;
        c_object.color = cv::Scalar(b, g, r);
        c_object.visualize = true;

        object_map_ptr_->insert({c_object.p_object.id, c_object});
    }
    ROS_INFO_STREAM("Done constructing objects");
}

bool label_model::has_next_image()
{
    while (bag_iter_ != v_->end())
    {
        rosbag::MessageInstance message = *bag_iter_;

        if (message.getTopic() == "/stereo/right/image_raw")
        {
            last_image_ = message.instantiate<sensor_msgs::Image>();
            ++bag_iter_;
            return true;
        }
        else
        {
            last_odom_ = message.instantiate<nav_msgs::Odometry>();
        }
        ++bag_iter_;
    }

    return false;
}
cv::Mat label_model::get_next_image()
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(last_image_, sensor_msgs::image_encodings::BGR8);
    cv::Mat cv_image = cv_ptr->image;
    const Eigen::Affine3d e_enu_to_baselink = odom_to_eigen(last_odom_).inverse();
    const Eigen::Affine3d e_enu_to_stereoright = e_baselink_to_stereoright_ * e_enu_to_baselink;

    for (const auto& id_course_object_pair : *object_map_ptr_)
    {
        const course_object& c_object = id_course_object_pair.second;

        if (!c_object.visualize)
        {
            continue;
        }

        const std::vector<Eigen::Vector4d> b = c_object.bounding_box_points;
        const Eigen::Vector4d position = b[0];
        const auto position_baselink = e_enu_to_baselink * position;

        const Eigen::Vector2d axis(1.0, 0.0);
        Eigen::Vector2d obj_2d_pos(position_baselink[0], position_baselink[1]);
        obj_2d_pos.normalize();

        const double similarity_score = axis.dot(obj_2d_pos);
        // If the points in baselink are in front of the boat, and within an angle threshold
        if (position_baselink[0] > 2.0 && similarity_score > .9)
        {
            // Convert the points to image coordinates, and draw said point on the image.
            Eigen::MatrixXd points(4, 8);
            points.col(0) = b[1];
            points.col(1) = b[2];
            points.col(2) = b[3];
            points.col(3) = b[4];
            points.col(4) = b[5];
            points.col(5) = b[6];
            points.col(6) = b[7];
            points.col(7) = b[8];

            const Eigen::Matrix4d ee = e_enu_to_stereoright.matrix();

            points = intrinsics_ * ee * points;
            points.row(0) = points.row(0).cwiseQuotient(points.row(2));
            points.row(1) = points.row(1).cwiseQuotient(points.row(2));

            const Eigen::MatrixXd min_matrix = points.rowwise().minCoeff();
            const Eigen::MatrixXd max_matrix = points.rowwise().maxCoeff();

            int min_x = min_matrix(0, 0);
            int min_y = min_matrix(1, 0);
            int max_x = max_matrix(0, 0);
            int max_y = max_matrix(1, 0);

            min_x = std::min(std::max(0, min_x), cv_image.cols);
            min_y = std::min(std::max(0, min_y), cv_image.rows);
            max_x = std::min(std::max(0, max_x), cv_image.cols);
            max_y = std::min(std::max(0, max_y), cv_image.rows);

            for (int i = 0; i != points.cols(); ++i)
            {
                const auto& p = points.col(i);
                cv::Mat roi = cv_image(cv::Rect(cv::Point(min_x, min_y), cv::Point(max_x, max_y)));
                cv::Mat color(roi.size(), CV_8UC3, c_object.color);
                double alpha = 0.05;
                cv::addWeighted(color, alpha, roi, 1.0 - alpha, 0.0, roi);
                cv::putText(cv_image, std::to_string(c_object.p_object.id) + ":" + c_object.p_object.classification,
                            cv::Point(min_x, max_y - 10), 0, 0.5, cv::Scalar(255, 255, 255));
            }
        }
    }
    return cv_image;
}

void label_model::restart()
{
    // TODO(tbianchi) put all the topics as consts
    std::vector<std::string> image_topics = {"/odom", "/stereo/right/image_raw"};
    bag_iter_ = v_->begin();
}


void label_model::generate_bag()
{
    rosbag::Bag bag_new;
    bag_new.open(output_bag_name_, rosbag::bagmode::Write);

    std::vector<std::string> topics;
    topics.push_back(std::string("/velodyne_points"));
    topics.push_back(std::string("/odom"));

    rosbag::View view(bag_, rosbag::TopicQuery(topics));
    std::vector<std::string> odom_topics = {"/odom"};
    rosbag::View view_odom(bag_, rosbag::TopicQuery(odom_topics));

    nav_msgs::Odometry::ConstPtr last_odom = view_odom.begin()->instantiate<nav_msgs::Odometry>();

    std::vector<mil_msgs::PerceptionObject> objects;
    for (const auto& id_object_pair : *object_map_ptr_)
    {
        objects.push_back(id_object_pair.second.p_object);
    }

    const int number_pcloud_messages = view.size() - view_odom.size();

    std::cout << "\nSAVING BAG" << std::endl;


    //TODO(tbianchi) Make this a popup thingy
    boost::progress_display show_progress(number_pcloud_messages);

    for (rosbag::MessageInstance& message : view)
    {
        if (message.getTopic() == "/velodyne_points")
        {
            auto object_copy = objects;

            const sensor_msgs::PointCloud2::ConstPtr point_cloud2 = message.instantiate<sensor_msgs::PointCloud2>();
            const Eigen::Affine3d e_baselink_to_enu = odom_to_eigen(last_odom);
            const Eigen::Affine3d e_velodyne_to_enu = e_baselink_to_enu * e_velodyne_to_baselink_;
            const auto point_cloud = pcodar::transform_point_cloud(*point_cloud2, e_velodyne_to_enu);

            mil_msgs::PerceptionObjectArray objects_in_frame;
            mil_msgs::PerceptionObject spurious_points;

            // TODO(tbianchi) fix this so that it uses smart enum
            spurious_points.classification = "spurious";

            std::set<uint32_t> indices;
            for (const auto& p : point_cloud.points)
            {
                geometry_msgs::Point32 g_p;
                g_p.x = p.x;
                g_p.y = p.y;
                g_p.z = p.z;
                int index_of_object = get_index_of_object(g_p, object_copy);
                if (index_of_object == -1)
                {
                    spurious_points.points.push_back(g_p);
                }
                else
                {
                    object_copy[index_of_object].points.push_back(g_p);
                    indices.insert(index_of_object);
                }
            }
            objects_in_frame.objects.push_back(spurious_points);
            for (const uint32_t ind : indices)
            {
                objects_in_frame.objects.push_back(object_copy[ind]);
            }

            bag_new.write("labels", message.getTime(), objects_in_frame);
            ++show_progress;
        }
        else
        {
            last_odom = message.instantiate<nav_msgs::Odometry>();
        }
    }
    bag_new.close();
}
