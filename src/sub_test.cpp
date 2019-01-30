#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <ros2_cpp_py/sub_test.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;


SubTest::SubTest() : Node("sub_test")
  {
    RCLCPP_INFO(get_logger(), "sub test");
    get_parameter_or("update_period", update_period_, update_period_);
    set_parameter_if_not_set("update_period", update_period_);
    timer_ = this->create_wall_timer(4s, std::bind(&SubTest::update, this));
    sub_ = create_subscription<sensor_msgs::msg::Image>("image",
        std::bind(&SubTest::callback, this, _1));
  }

SubTest::~SubTest()
  {
    RCLCPP_INFO(get_logger(), "sub test shutting down");
  }

void SubTest::callback(sensor_msgs::msg::Image::SharedPtr msg)
  {
    stamps_.push_back(msg->header.stamp);
  }

void SubTest::update()
  {
    auto cur = now();

    if (stamps_.size() < 2) {
      RCLCPP_WARN(get_logger(), "Not enough received messages");
      return;
    }

    // TODO(lucasw) make these threshold configurable
    while ((((cur - stamps_.front()).nanoseconds() > update_period_ * 1e9) && (stamps_.size() > 50)) ||
           (stamps_.size() > 200)) {
      stamps_.pop_front();
    }

    auto last_diff = cur - stamps_.back();
    if (last_diff.nanoseconds() > update_period_ * 1e9) {
      RCLCPP_INFO(get_logger(), "time since last message %f", last_diff.nanoseconds() / 1e9);
    } else {
      const double rate = static_cast<double>(stamps_.size()) / ((cur - stamps_.front()).nanoseconds() / 1e9);
      // for (auto stamp : stamps) {
      //
      // }
      RCLCPP_INFO(get_logger(), "messages per second %f", rate);
    }
  }

#include <class_loader/register_macro.hpp>

CLASS_LOADER_REGISTER_CLASS(SubTest, rclcpp::Node)
