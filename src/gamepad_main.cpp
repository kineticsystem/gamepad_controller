#include <gamepad/gamepad_node.hpp>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<gamepad::GamepadNode>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  exec.remove_node(node);
  rclcpp::shutdown();
}
