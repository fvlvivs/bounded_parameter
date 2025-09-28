#include "bounded_parameter/bounded_parameter.hpp"
#include <memory>
#include <rclcpp/rclcpp.hpp>

class ExampleNode : public rclcpp::Node {
public:
  ExampleNode() : Node("example_node") {

    declareBoundedParameter<int>(this, "bounded_int");
    declareBoundedParameter<double>(this, "bounded_double");
    declareBoundedParameter<std::string>(this, "bounded_string");

    // Create a timer to demonstrate parameter usage
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&ExampleNode::timer_callback, this));
  }

private:
  void timer_callback() {
    int int_value;
    if (getBoundedParameter(this, "bounded_int", int_value)) {
      RCLCPP_INFO(this->get_logger(), "Bounded integer value: %d", int_value);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Integer parameter out of bounds! Using default value %d.",
                  int_value);
    }

    double double_value;
    if (getBoundedParameter(this, "bounded_double", double_value)) {
      RCLCPP_INFO(this->get_logger(), "Bounded double value: %.2f",
                  double_value);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Double parameter out of bounds! Using default value %.2f.",
                  double_value);
    }

    std::string string_value;
    if (getBoundedParameter(this, "bounded_string", string_value)) {
      RCLCPP_INFO(this->get_logger(), "Valid string value: %s",
                  string_value.c_str());
    } else {
      RCLCPP_WARN(
          this->get_logger(),
          "String parameter not in allowed values! Using default value: %s.",
          string_value.c_str());
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExampleNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
