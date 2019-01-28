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
    timer_ = this->create_wall_timer(1s, std::bind(&SubTest::update, this));
    sub_ = create_subscription<sensor_msgs::msg::Image>("image",
        std::bind(&SubTest::callback, this, _1));
  }

  ~SubTest()
  {
    std::cout << "sub test shutting down\n";
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
      std::cout << "Not enough received messages\n";
      return;
    }

    // TODO(lucasw) make these threshold configurable
    while ((((cur - stamps_.front()).nanoseconds() > 2e9) && (stamps_.size() > 50)) ||
           (stamps_.size() > 200)) {
      stamps_.pop_front();
    }

    auto last_diff = cur - stamps_.back();
    if (last_diff.nanoseconds() > 1e9) {
      std::cout << last_diff.nanoseconds() / 1e9 << " since last message\n";
    } else {
      const double rate = static_cast<double>(stamps_.size()) / ((cur - stamps_.front()).nanoseconds() / 1e9);
      // for (auto stamp : stamps) {
      //
      // }
      std::cout << "messages per second " << rate << "\n";
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
