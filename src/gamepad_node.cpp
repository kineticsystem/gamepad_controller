#include <gamepad/gamepad_node.hpp>

namespace gamepad {
constexpr auto kLogger = "GamepadNode";

GamepadNode::GamepadNode() : Node("gamepad") {
  buttons_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10,
      std::bind(&GamepadNode::callback, this, std::placeholders::_1));
}

void GamepadNode::callback(
    [[maybe_unused]] const sensor_msgs::msg::Joy::SharedPtr msg) {

  for (size_t i = 0; i < msg->buttons.size(); ++i) {
    if (msg->buttons[i]) {
      RCLCPP_INFO(rclcpp::get_logger(kLogger), "Button %zu pressed", i);
    }
  }
}
} // namespace gamepad
