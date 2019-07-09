#include <QApplication>
#include "UI.h"

int main(int argc, char *argv[])
{
  // Ros init
  ros::init(argc, argv, "judges_panel");
  ros::Time::init();

  // Check that ROS is alive before continuing... After 10 minutes quit!
  ROS_INFO("JUDGEPANEL | Checking ROS master is alive...");
  ros::Time rostimer = ros::Time::now();
  while (!ros::master::check())
  {
    if ((ros::Time::now() - rostimer).toSec() > 600)
    {
      return -1;
    }
    ros::Duration(0.1).sleep();
  }
  ROS_INFO_STREAM("JUDGEPANEL | ROS Master: " << ros::master::getHost());
  QApplication a(argc, argv);
  UI form;
  form.show();

  return a.exec();
}
