#include <deque>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>


class SubTest : public rclcpp::Node
{
public:
  SubTest();
   ~SubTest();

private:
  // TODO(lucasw) maybe keep the entire messages and analyze bandwidth
  std::deque<rclcpp::Time> stamps_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  void callback(sensor_msgs::msg::Image::SharedPtr msg);

  rclcpp::TimerBase::SharedPtr timer_;
  void update();

  float update_period_ = 4.0;
};
