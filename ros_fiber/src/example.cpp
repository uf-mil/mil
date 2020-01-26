#include "ros_fiber.hpp"
#include "std_msgs/String.h"
#include <actionlib_tutorials/FibonacciAction.h>

int main(int argc, char** argv)
{
  ros_fiber::init(argc, argv, "ros_fiber_example");
  ros_fiber::NodeHandle nh;
  boost::fibers::fiber main([&nh]() {
    auto fsub = nh.subscribe<std_msgs::String>("chatter", 1);
    auto fsub2 = nh.subscribe<std_msgs::String>("chatter2", 1);
    auto pub = nh.advertise<std_msgs::String>("output", 1);
    auto action_client = nh.actionClient<actionlib_tutorials::FibonacciAction>("/fibonacci");
    boost::fibers::fiber f1([&fsub]() {
      while (ros::ok()) {
        auto msg = fsub.get_next_message();
        std::cout << "Chatter1 said " << msg << std::endl;
      }
    });
    boost::fibers::fiber f2([&fsub2, &nh, &pub] () {
      while(ros::ok()) {
        auto msg = fsub2.get_next_message();
        std::cout << "Chatter2 said " << msg << std::endl;
        nh.sleep(ros::Duration(1));
        pub.publish(msg);
      }
    });
    boost::fibers::fiber f3([&action_client] () {
      actionlib_tutorials::FibonacciGoal goal;
      goal.order = 2;
      auto res = action_client.sendGoal(goal)->get_result();
      std::cout << "res " << res.state.toString() << " " << res.result << std::endl;
    });

    f3.join();
    f2.join();
    f1.join();
  });
  printf("%s:%s:%d\n", __FILE__, __PRETTY_FUNCTION__, __LINE__);
  ros_fiber::spin();
  main.detach();
  printf("%s:%s:%d\n", __FILE__, __PRETTY_FUNCTION__, __LINE__);
}
