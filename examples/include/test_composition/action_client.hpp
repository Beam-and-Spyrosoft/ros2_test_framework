#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <test_composition/action/move_robot.hpp>

namespace test_composition {

class ActionClient : public rclcpp::Node {
public:
  using MoveRobot = test_composition::action::MoveRobot;
  using GoalHandleMoveRobot = rclcpp_action::ClientGoalHandle<MoveRobot>;

  explicit ActionClient(const rclcpp::NodeOptions& options);

  void send_goal(float x, float y);
  void cancel_current_goal();

  // For testing
  bool has_active_goal() const { return current_goal_handle_ != nullptr; }
  bool get_last_result_success() const { return last_result_success_; }
  std::string get_last_result_message() const { return last_result_message_; }
private:
  void goal_response_callback(const GoalHandleMoveRobot::SharedPtr& goal_handle);
  void feedback_callback(
    GoalHandleMoveRobot::SharedPtr,
    const std::shared_ptr<const MoveRobot::Feedback> feedback);
  void result_callback(const GoalHandleMoveRobot::WrappedResult& result);
  rclcpp_action::Client<MoveRobot>::SharedPtr action_client_;
  GoalHandleMoveRobot::SharedPtr current_goal_handle_;

  // Last result data
  bool last_result_success_{false};
  std::string last_result_message_;
};

} // namespace test_composition