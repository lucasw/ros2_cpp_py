#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <ros2_cpp_py/pub_test.hpp>
#include <ros2_cpp_py/sub_test.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  auto publisher_node = std::make_shared<Color>();
  auto subscriber_node1 = std::make_shared<SubTest>();
  auto subscriber_node2 = std::make_shared<SubTest>();
  exec.add_node(publisher_node);
  exec.add_node(subscriber_node1);
  exec.add_node(subscriber_node2);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
