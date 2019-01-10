// Jacob Beck

#include <chrono>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

#if 0
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
#endif

class CppTest : public rclcpp::Node {
 public:
  CppTest() : Node("cpptest"), count_(0) {
    int test = 0;
    get_parameter_or("test", test, 43);
    set_parameter_if_not_set("test", test);
    RCLCPP_INFO(get_logger(), "test param value %d", test);
    timer_ = this->create_wall_timer(3s, std::bind(&CppTest::timer_callback_, this));

#if 0
    parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this);
    param_sub_ = parameters_client_->on_parameter_event(
        std::bind(&CppTest::onParameterEvent, this, _1));
#endif
  }

  ~CppTest() {
    std::cout << "cpp test shutting down" << std::endl;
  }

  void init() {
#if 0
    object_with_node_ = std::make_shared<ObjectWithNode>(shared_from_this());
#endif

    // TODO(lucasw) does the name here do anything?
    // const std::string full_name = std::string(get_namespace()) + std::string(get_name());
    // parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, full_name);
    #if 0
    parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this);
    param_sub_ = parameters_client_->on_parameter_event(
        std::bind(&CppTest::onParameterEvent, this, _1));
    #endif
  }

 private:
  void timer_callback_() {
    RCLCPP_DEBUG(this->get_logger(), "Hello! %d", count_);
    count_++;
  }

#if 0
  std::shared_ptr<ObjectWithNode> object_with_node_;
#endif
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;

  rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr param_sub_;
  void onParameterEvent(const rcl_interfaces::msg::ParameterEvent::SharedPtr event) {
    const std::string full_name = std::string(get_namespace()) + std::string(get_name());
    std::cout << full_name << ", event node: " << event->node << "\n";

    std::cout << "new:\n ";
    for (auto param : event->new_parameters) {
      std::cout << param.name << " ";
    }
    std::cout << "\nchanged:\n ";
    for (auto param : event->changed_parameters) {
      std::cout << param.name << " ";
    }
    std::cout << "\ndeleted:\n ";
    for (auto param : event->deleted_parameters) {
      std::cout << param.name << " ";
    }
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto cpp_test = std::make_shared<CppTest>();
  cpp_test->init();
  rclcpp::spin(cpp_test);
  rclcpp::shutdown();
  return 0;
}
