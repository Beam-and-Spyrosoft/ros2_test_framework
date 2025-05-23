#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <test_composition/action/move_robot.hpp>

namespace test_composition {

class ActionServer : public rclcpp::Node {
public:
  using MoveRobot = test_composition::action::MoveRobot;
  using GoalHandleMoveRobot = rclcpp_action::ServerGoalHandle<MoveRobot>;

  explicit ActionServer(const rclcpp::NodeOptions& options);

  // For testing access
  bool is_moving() const { return is_moving_; }
  std::pair<float, float> get_current_position() const { return {current_x_, current_y_}; }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const MoveRobot::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleMoveRobot> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleMoveRobot> goal_handle);

  void execute_move(const std::shared_ptr<GoalHandleMoveRobot> goal_handle);

  rclcpp_action::Server<MoveRobot>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr timer_;

  // State
  bool is_moving_{false};
  float current_x_{0.0};
  float current_y_{0.0};
  std::shared_ptr<GoalHandleMoveRobot> current_goal_handle_;
};

} // namespace test_composition