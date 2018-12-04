// Jacob Beck

#include <chrono>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class ObjectWithNode {
public:
  // These are equivalent?
  ObjectWithNode(std::shared_ptr<rclcpp::Node> node) : node_(node) {}
  // ObjectWithNode(rclcpp::Node::SharedPtr node) : node_(node) {}

  ~ObjectWithNode() {
    std::cout << "object with node shutting down" << std::endl;
  }
private:
  std::shared_ptr<rclcpp::Node> node_;
  // rclcpp::Node::SharedPtr node_;
};

class CppTest : public rclcpp::Node {
 public:
  CppTest() : Node("cpptest"), count_(0) {
    timer_ = this->create_wall_timer(1s, std::bind(&CppTest::timer_callback_, this));
  }

  ~CppTest() {
    std::cout << "cpp test shutting down" << std::endl;
  }

  void init() {
    object_with_node_ = std::make_shared<ObjectWithNode>(shared_from_this());
  }

 private:
  void timer_callback_() {
    RCLCPP_INFO(this->get_logger(), "Hello! %d", count_);
    count_++;
  }

  std::shared_ptr<ObjectWithNode> object_with_node_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto cpp_test = std::make_shared<CppTest>();
  // cpp_test->init();
  rclcpp::spin(cpp_test);
  rclcpp::shutdown();
  return 0;
}
