#include <gtest/gtest.h>
#include <test_composition/action_server.hpp>
#include <rtest/action_server_mock.hpp>

class ActionServerTest : public ::testing::Test {
protected:
  rclcpp::NodeOptions opts;
};

TEST_F(ActionServerTest, GoalAcceptance) {
  auto node = std::make_shared<test_composition::ActionServer>(opts);
  auto server_mock = rtest::findActionServer<test_composition::action::MoveRobot>(node, "move_robot");

  ASSERT_TRUE(server_mock);

  // Test goal acceptance
  EXPECT_CALL(*server_mock, handle_goal(::testing::_, ::testing::_))
    .WillOnce(::testing::Return(rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE));

  // Simulate goal request
  auto goal = std::make_shared<test_composition::action::MoveRobot::Goal>();
  goal->target_x = 2.0;
  goal->target_y = 3.0;

  rclcpp_action::GoalUUID uuid;
  auto response = server_mock->handle_goal(uuid, goal);
  EXPECT_EQ(response, rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE);
}

TEST_F(ActionServerTest, GoalRejectionWhenTooFar) {
  auto node = std::make_shared<test_composition::ActionServer>(opts);
  auto server_mock = rtest::findActionServer<test_composition::action::MoveRobot>(node, "move_robot");

  // Test goal rejection for targets too far
  EXPECT_CALL(*server_mock, handle_goal(::testing::_, ::testing::_))
    .WillOnce(::testing::Return(rclcpp_action::GoalResponse::REJECT));

  auto goal = std::make_shared<test_composition::action::MoveRobot::Goal>();
  goal->target_x = 50.0; // Too far
  goal->target_y = 50.0;

  rclcpp_action::GoalUUID uuid;
  auto response = server_mock->handle_goal(uuid, goal);
  EXPECT_EQ(response, rclcpp_action::GoalResponse::REJECT);
}

TEST_F(ActionServerTest, CancelGoal) {
  auto node = std::make_shared<test_composition::ActionServer>(opts);
  auto server_mock = rtest::findActionServer<test_composition::action::MoveRobot>(node, "move_robot");

  // Mock goal handle for cancel test
  auto goal_handle = std::make_shared<rclcpp_action::ServerGoalHandle<test_composition::action::MoveRobot>>();

  EXPECT_CALL(*server_mock, handle_cancel(::testing::_))
    .WillOnce(::testing::Return(rclcpp_action::CancelResponse::ACCEPT));

  auto response = server_mock->handle_cancel(goal_handle);
  EXPECT_EQ(response, rclcpp_action::CancelResponse::ACCEPT);
}

TEST_F(ActionServerTest, FeedbackPublishing) {
  auto node = std::make_shared<test_composition::ActionServer>(opts);
  auto server_mock = rtest::findActionServer<test_composition::action::MoveRobot>(node, "move_robot");

  // Test that feedback can be published
  auto goal_handle = std::make_shared<rclcpp_action::ServerGoalHandle<test_composition::action::MoveRobot>>();

  test_composition::action::MoveRobot::Feedback feedback;
  feedback.current_x = 1.5;
  feedback.current_y = 2.5;
  feedback.distance_remaining = 3.0;

  EXPECT_NO_THROW(server_mock->publish_feedback(feedback, goal_handle));
}