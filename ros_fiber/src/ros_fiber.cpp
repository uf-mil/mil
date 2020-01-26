#include <signal.h>

namespace ros_fiber
{

namespace {

boost::fibers::unbuffered_channel<int> signal_channel;
void signal_cb(int signal)
{
  boost::fibers::fiber([signal] () {
    signal_channel.push(signal);
  });
}

}

template<typename T>
class Subscriber
{
public:
  using promise_t = boost::fibers::promise<T>;
  using future_t = boost::fibers::future<T>;
  using MessageT = T;

  RosFiberSubscriber(ros::NodeHandle& nh, std::string topic, uint32_t queue_size)
  {
    sub = nh.subscribe(topic, queue_size, &::callback, this);
  }

  future_t get_next_message()
  {
    promise_t promise;
    future_t future = promise.get_future();
    next_message_promises_.emplace_back(std::move(promise));
    return future;
  }

private:
  void callback(T msg)
  {
    for (auto& promise : next_message_promises_)
    {
      promise.set_value(msg);
    }
    next_message_promises_.clear();
  }
  std::vector<promise_t> next_message_promises_;
  ros::Subscriber sub;
};


boost::fibers::promise<ros::TimerEvent> ros_fiber_timer(ros::Duration period, bool oneshot = false)
{
  boost::fibers::promise<ros::TimerEvent> promise;
  boost::fibers::fiber f([promise]() {
  }); 
  return promise;
}



}

