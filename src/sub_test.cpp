#include <chrono>
#include <deque>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

using namespace std::chrono_literals;
using std::placeholders::_1;


class SubTest
{
public:
  SubTest()
  {
    ROS_INFO("sub test");
    timer_ = nh_.createTimer(ros::Duration(update_period_),
        &SubTest::update, this);
    sub_ = nh_.subscribe<sensor_msgs::Image>("image", 10,
        &SubTest::callback, this);
  }

  ~SubTest()
  {
    ROS_INFO("sub test shutting down");
  }

private:
  ros::NodeHandle nh_;
  // TODO(lucasw) maybe keep the entire messages and analyze bandwidth
  std::deque<ros::Time> stamps_;

  ros::Subscriber sub_;
  void callback(const sensor_msgs::ImageConstPtr& msg)
  {
    stamps_.push_back(msg->header.stamp);
  }

  const float update_period_ = 4.0;
  ros::Timer timer_;
  void update(const ros::TimerEvent& e)
  {
    (void)e;
    auto cur = ros::Time::now();

    if (stamps_.size() < 2) {
      ROS_WARN("Not enough received messages");
      return;
    }

    // TODO(lucasw) make these threshold configurable
    while ((((cur - stamps_.front()).toSec() > update_period_) && (stamps_.size() > 50)) ||
           (stamps_.size() > 200)) {
      stamps_.pop_front();
    }

    auto last_diff = cur - stamps_.back();
    if (last_diff.toSec() > 4.0) {
      ROS_INFO("time since last message %f", last_diff.toSec());
    } else {
      const double rate = static_cast<double>(stamps_.size()) / ((cur - stamps_.front()).toSec());
      // for (auto stamp : stamps) {
      //
      // }
      ROS_INFO("messages per second %f", rate);
    }
  }
};

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "sub_test");
  auto sub_test = std::make_shared<SubTest>();
  ros::spin();
  return 0;
}
