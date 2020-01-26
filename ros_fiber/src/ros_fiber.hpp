#include <signal.h>
#include <ros/ros.h>
#include <ros/xmlrpc_manager.h>
#include <actionlib/client/simple_action_client.h>
#include <boost/fiber/all.hpp>

namespace ros_fiber
{

namespace {

const int ROSNODE_KILL_SIGNAL = -1;
boost::fibers::unbuffered_channel<int> shutdown_channel;
void signal_cb(int signal)
{
  boost::fibers::fiber([signal] () {
    shutdown_channel.push(signal);
  }).join();
}

}

template<typename T>
class Subscriber
{
public:
  using promise_t = boost::fibers::promise<T>;
  using future_t = boost::fibers::future<T>;
  using MessageT = T;

  Subscriber(ros::NodeHandle& nh, std::string topic, uint32_t queue_size)
  {
    sub = nh.subscribe(topic, queue_size, &Subscriber::callback, this);
  }

  T get_next_message()
  {
    promise_t promise;
    future_t future = promise.get_future();
    next_message_promises_.emplace_back(std::move(promise));
    return future.get();
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
  //boost::fibers::mutex promises_guard_;
  std::vector<promise_t> next_message_promises_;
  ros::Subscriber sub;
};

template<class ActionSpec>
class SimpleActionClient;


/// TODO: handle feedback
template<class ActionSpec>
class SimpleActionGoal 
{
public:
  ACTION_DEFINITION(ActionSpec)

  typedef struct {
    actionlib::SimpleClientGoalState state;
    Result result;
  } done_t;
  using done_promise_t = boost::fibers::promise<done_t>;

  done_t get_result()
  {
    return done_promise_.get_future().get();
  }

  SimpleActionGoal() = default;
private:
  // Make SimpleActionClient a friend so it can construct and handle promises
  friend SimpleActionClient<ActionSpec>;  
  done_promise_t done_promise_;
};

template<class ActionSpec>
class SimpleActionClient
{
public:
  ACTION_DEFINITION(ActionSpec)

  SimpleActionClient(ros::NodeHandle& nh, const std::string& name) :
    action_client_(std::make_shared<action_client_t>(nh, name, true))
  {

  }

  using goal_t = SimpleActionGoal<ActionSpec>;

  std::shared_ptr<goal_t> sendGoal(Goal const& goal)
  {
    auto ret = std::make_shared<goal_t>();
      printf("%s:%s:%d\n", __FILE__, __PRETTY_FUNCTION__, __LINE__);
    action_client_->sendGoal(goal, 
      [ret](const actionlib::SimpleClientGoalState & state, boost::shared_ptr<const Result> const& result) {
          printf("%s:%s:%d\n", __FILE__, __PRETTY_FUNCTION__, __LINE__);
        typename goal_t::done_t done_value{state, *result};
        ret->done_promise_.set_value(done_value);
      }
    );
    ros::spin();
      printf("%s:%s:%d\n", __FILE__, __PRETTY_FUNCTION__, __LINE__);
    return ret;
  }
private:
  using action_client_t = actionlib::SimpleActionClient<ActionSpec>;
  std::shared_ptr<action_client_t> action_client_;
};


class NodeHandle
{
public:
  template <typename ...Args>
  NodeHandle(Args && ...args) : nh(std::forward<Args>(args)...)
  {
  }

  template<class T>
  Subscriber<T> subscribe(std::string topic, uint32_t queue_size)
  {
    return Subscriber<T>(this->nh, topic, queue_size);
  }

  template <typename msg_t, typename ...Args>
  ros::Publisher  advertise(Args && ...args)
  {
    return nh.advertise<msg_t>(std::forward<Args>(args)...);
  }

  template<class ActionSpec>
  SimpleActionClient<ActionSpec> actionClient(const std::string& name)
  {
    return std::move(SimpleActionClient<ActionSpec>(nh, name));
  }

  void sleep(ros::Duration d)
  {
    boost::fibers::promise<void> promise;
    auto future = promise.get_future();
    auto timer = nh.createTimer(d, [&promise](ros::TimerEvent const& event){
      promise.set_value();
    }, true);
    future.get();
  }
private:
  ros::NodeHandle nh;
};

void init(int argc, char** argv, std::string node)
{
  signal(SIGINT, signal_cb);
  ros::init(argc, argv, node, ros::init_options::NoSigintHandler);
  ros::XMLRPCManager::instance()->unbind("shutdown");
  ros::XMLRPCManager::instance()->bind("shutdown", [](XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result) {
    boost::fibers::fiber([] () {
      shutdown_channel.push(ROSNODE_KILL_SIGNAL);
    }).join();
  });
}

void spin(int ros_threads = 1)
{
  ros::AsyncSpinner spinner(ros_threads);
  spinner.start();
  boost::fibers::fiber signal_waiter([]() {
      int my_sig = -1;
      shutdown_channel.pop(my_sig);
      std::cout << "signal is " << my_sig << std::endl;
  });
  printf("%s:%s:%d\n", __FILE__, __PRETTY_FUNCTION__, __LINE__);
  signal_waiter.join();
  printf("%s:%s:%d\n", __FILE__, __PRETTY_FUNCTION__, __LINE__);
  ros::shutdown();
  printf("%s:%s:%d\n", __FILE__, __PRETTY_FUNCTION__, __LINE__);
}

}

