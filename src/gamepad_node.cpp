#include <gamepad/gamepad_node.hpp>

namespace gamepad {

constexpr auto kLogger = "GamepadNode";
constexpr auto kDoObjectiveActionName = "do_objective";
constexpr double kActionGoalResponseWaitSeconds = 3;

GamepadNode::GamepadNode() : Node("gamepad") {
  buttons_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10,
      std::bind(&GamepadNode::callback, this, std::placeholders::_1));

  do_objective_client_ = rclcpp_action::create_client<
      moveit_studio_agent_msgs::action::DoObjectiveSequence>(
      this, kDoObjectiveActionName);

  execute_objective_client_ =
      this->create_client<moveit_studio_agent_msgs::srv::ExecuteObjective>(
          "/execute_objective");
  cancel_objective_client_ =
      this->create_client<moveit_studio_agent_msgs::srv::CancelObjective>(
          "/cancel_objective");
}

void GamepadNode::callback(
    [[maybe_unused]] const sensor_msgs::msg::Joy::SharedPtr msg) {

  // Initialize last_buttons_ vector if it's the first message
  if (last_buttons_.empty()) {
    last_buttons_.resize(msg->buttons.size(), 0);
  }

  for (size_t i = 0; i < msg->buttons.size(); ++i) {
    if (msg->buttons[i] && !last_buttons_[i]) {
      RCLCPP_INFO(rclcpp::get_logger(kLogger), "Button %zu pressed", i);
      if (i == 0) {
        send_objective("Teleoperate");
      } else if (i == 2) {
        cancel_objective();
      } else if (i == 3) {
        send_objective("Close Gripper");
        send_objective("Teleoperate");
      } else if (i == 1) {
        send_objective("Open Gripper");
        send_objective("Teleoperate");
      }
    }
    last_buttons_[i] = msg->buttons[i];
  }
}

void GamepadNode::send_objective(std::string name) {

  auto request = std::make_shared<
      moveit_studio_agent_msgs::srv::ExecuteObjective::Request>();
  request->objective_name = name;

  auto result_future = execute_objective_client_->async_send_request(request);
  if (result_future.wait_for(std::chrono::duration<double>(
          kActionGoalResponseWaitSeconds)) == std::future_status::timeout) {
    std::cout
        << "Failed to send action to server: timed out waiting for a response. "
        << std::to_string(kActionGoalResponseWaitSeconds) << " seconds."
        << std::endl;
  }
}

void GamepadNode::cancel_objective() {

  auto request = std::make_shared<
      moveit_studio_agent_msgs::srv::CancelObjective::Request>();

  auto result_future = cancel_objective_client_->async_send_request(request);
  if (result_future.wait_for(std::chrono::duration<double>(
          kActionGoalResponseWaitSeconds)) == std::future_status::timeout) {
    std::cout
        << "Failed to send action to server: timed out waiting for a response. "
        << std::to_string(kActionGoalResponseWaitSeconds) << " seconds."
        << std::endl;
  }
}

} // namespace gamepad
