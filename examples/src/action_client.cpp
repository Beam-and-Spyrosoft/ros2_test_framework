#include "test_composition/action_client.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace test_composition {

ActionClient::ActionClient(const rclcpp::NodeOptions& options)
: rclcpp::Node("move_robot_client", options) {
  action_client_ = rclcpp_action::create_client<MoveRobot>(this, "move_robot");
  RCLCPP_INFO(get_logger(), "Move robot action client started");
}

void ActionClient::send_goal(float x, float y) {
  if (!action_client_->action_server_is_ready()) {
    RCLCPP_ERROR(get_logger(), "Action server not available");
    return;
  }

  auto goal_msg = MoveRobot::Goal();
  goal_msg.target_x = x;
  goal_msg.target_y = y;

  RCLCPP_INFO(get_logger(), "Sending goal: move to (%.2f, %.2f)", x, y);

  auto send_goal_options = rclcpp_action::Client<MoveRobot>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&ActionClient::goal_response_callback, this, std::placeholders::_1);
  send_goal_options.feedback_callback =
    std::bind(&ActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(&ActionClient::result_callback, this, std::placeholders::_1);

  action_client_->async_send_goal(goal_msg, send_goal_options);
}

void ActionClient::cancel_current_goal() {
  if (current_goal_handle_) {
    RCLCPP_INFO(get_logger(), "Canceling current goal");
    action_client_->async_cancel_goal(current_goal_handle_);
  }
}

void ActionClient::goal_response_callback(const GoalHandleMoveRobot::SharedPtr& goal_handle) {
  if (!goal_handle) {
    RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
    current_goal_handle_ = nullptr;
  } else {
    RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result");
    current_goal_handle_ = goal_handle;
  }
}

void ActionClient::feedback_callback(
  GoalHandleMoveRobot::SharedPtr,
  const std::shared_ptr<const MoveRobot::Feedback> feedback) {

  RCLCPP_INFO(get_logger(),
    "Feedback: current position (%.2f, %.2f), distance remaining: %.2f",
    feedback->current_x, feedback->current_y, feedback->distance_remaining);
}

void ActionClient::result_callback(const GoalHandleMoveRobot::WrappedResult& result) {
  current_goal_handle_ = nullptr;

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      last_result_success_ = result.result->success;
      last_result_message_ = result.result->message;
      RCLCPP_INFO(get_logger(), "Goal succeeded: %s", result.result->message.c_str());
      break;
    case rclcpp_action::ResultCode::ABORTED:
      last_result_success_ = false;
      last_result_message_ = "Goal aborted";
      RCLCPP_ERROR(get_logger(), "Goal aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      last_result_success_ = false;
      last_result_message_ = "Goal canceled";
      RCLCPP_INFO(get_logger(), "Goal canceled");
      break;
    default:
      last_result_success_ = false;
      last_result_message_ = "Unknown result code";
      RCLCPP_ERROR(get_logger(), "Unknown result code");
      break;
  }
}

} // namespace test_composition

RCLCPP_COMPONENTS_REGISTER_NODE(test_composition::ActionClient)