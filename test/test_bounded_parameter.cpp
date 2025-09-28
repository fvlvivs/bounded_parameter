#include <gtest/gtest.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "bounded_parameter/bounded_parameter.hpp"

class TestBoundedParameter : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_node");

    declareBoundedParameter<int>(node_.get(), "bounded_int", 10, -100, 100, 42);
    declareBoundedParameter<double>(node_.get(), "bounded_double", 15.0, 10.0,
                                    30.0, 20.0);
    declareBoundedParameter<std::string>(
        node_.get(), "bounded_string", "map",
        std::vector<std::string>{"map", "odom", "base_link"}, "base_link");
  }

  void TearDown() override { rclcpp::shutdown(); }

  std::shared_ptr<rclcpp::Node> node_;
};

TEST_F(TestBoundedParameter, TestIntegerParameter) {

  int value;
  bool result = getBoundedParameter(node_.get(), "bounded_int", value);
  EXPECT_TRUE(result);
  EXPECT_EQ(value, 42);
}

TEST_F(TestBoundedParameter, TestIntegerOutOfBounds) {
  node_->set_parameter(rclcpp::Parameter("bounded_int.value", 200));

  int value;
  bool result = getBoundedParameter(node_.get(), "bounded_int", value);
  EXPECT_FALSE(result);
  EXPECT_EQ(value, 10);
}

TEST_F(TestBoundedParameter, TestDoubleParameter) {
  double value;
  bool result = getBoundedParameter(node_.get(), "bounded_double", value);
  EXPECT_TRUE(result);
  EXPECT_DOUBLE_EQ(value, 20.0);
}

TEST_F(TestBoundedParameter, TestDoubleOutOfBounds) {
  // Set value outside bounds
  node_->set_parameter(rclcpp::Parameter("bounded_double.value", -5.0));

  double value;
  bool result = getBoundedParameter(node_.get(), "bounded_double", value);
  EXPECT_FALSE(result);
  EXPECT_DOUBLE_EQ(value, 15.0);
}

TEST_F(TestBoundedParameter, TestStringParameter) {
  std::string value;
  bool result = getBoundedParameter(node_.get(), "bounded_string", value);
  EXPECT_TRUE(result);
  EXPECT_EQ(value, "base_link");
}

TEST_F(TestBoundedParameter, TestInvalidString) {
  node_->set_parameter(
      rclcpp::Parameter("bounded_string.value", "invalid_frame"));

  std::string value;
  bool result = getBoundedParameter(node_.get(), "bounded_string", value);
  EXPECT_FALSE(result);
  EXPECT_EQ(value, "map");
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
