#include <point_cloud_object_detection_and_recognition/pcodar_controller.hh>

int main(int argc, char* argv[])
{
    ROS_INFO_STREAM("Starting PCODAR node");
    
    pcodar::pcodar_controller c(argc, argv);
    c.initialize();
    c.execute();

    return 0;
}
