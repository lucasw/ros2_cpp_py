#include <chrono>
#include <deque>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;


class SubTest : public rclcpp::Node
{
public:
  SubTest() : Node("cpptest")
  {
    RCLCPP_INFO(get_logger(), "sub test");
    timer_ = this->create_wall_timer(4s, std::bind(&SubTest::update, this));
    sub_ = create_subscription<sensor_msgs::msg::Image>("image",
        std::bind(&SubTest::callback, this, _1));
  }

  ~SubTest()
  {
    RCLCPP_INFO(get_logger(), "sub test shutting down");
  }

private:
  // TODO(lucasw) maybe keep the entire messages and analyze bandwidth
  std::deque<rclcpp::Time> stamps_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  void callback(sensor_msgs::msg::Image::SharedPtr msg)
  {
    stamps_.push_back(msg->header.stamp);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  void update()
  {
    auto cur = now();

    if (stamps_.size() < 2) {
      RCLCPP_WARN(get_logger(), "Not enough received messages");
      return;
    }

    // TODO(lucasw) make these threshold configurable
    while ((((cur - stamps_.front()).nanoseconds() > 2e9) && (stamps_.size() > 50)) ||
           (stamps_.size() > 200)) {
      stamps_.pop_front();
    }

    auto last_diff = cur - stamps_.back();
    if (last_diff.nanoseconds() > 4e9) {
      RCLCPP_INFO(get_logger(), "time since last message %f", last_diff.nanoseconds() / 1e9);
    } else {
      const double rate = static_cast<double>(stamps_.size()) / ((cur - stamps_.front()).nanoseconds() / 1e9);
      // for (auto stamp : stamps) {
      //
      // }
      RCLCPP_INFO(get_logger(), "messages per second %f", rate);
    }
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto sub_test = std::make_shared<SubTest>();
  rclcpp::spin(sub_test);
  rclcpp::shutdown();
  return 0;
}
