#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <moveit_studio_agent_msgs/action/do_objective_sequence.hpp>
#include <moveit_studio_agent_msgs/srv/cancel_objective.hpp>
#include <moveit_studio_agent_msgs/srv/execute_objective.hpp>

namespace gamepad {

class GamepadNode : public rclcpp::Node {
public:
  GamepadNode();

private:
  void callback(const sensor_msgs::msg::Joy::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr buttons_sub_;

  // Stores the state of buttons from the last message
  std::vector<int> last_buttons_;

  rclcpp_action::Client<moveit_studio_agent_msgs::action::DoObjectiveSequence>::
      SharedPtr do_objective_client_;

  rclcpp::Client<moveit_studio_agent_msgs::srv::ExecuteObjective>::SharedPtr
      execute_objective_client_;
  rclcpp::Client<moveit_studio_agent_msgs::srv::CancelObjective>::SharedPtr
      cancel_objective_client_;

  void send_objective(std::string name);
  void cancel_objective();
};
} // namespace gamepad
