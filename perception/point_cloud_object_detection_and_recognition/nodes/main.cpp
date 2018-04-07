#include <point_cloud_object_detection_and_recognition/pcodar_controller.hpp>
#include <dynamic_reconfigure/client.h>
#include <navigator_tools/BoundsConfig.h>



int main(int argc, char* argv[])
{
    ROS_INFO_STREAM("Starting PCODAR node");
    pcodar::pcodar_controller c(argc, argv);
    c.initialize();
    c.execute();

    return 0;
}
