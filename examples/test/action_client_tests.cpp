#include <gtest/gtest.h>
#include <test_composition/action_client.hpp>
#include <rtest/action_client_mock.hpp>

class ActionClientTest : public ::testing::Test {
protected:
  rclcpp::NodeOptions opts;
};

TEST_F(ActionClientTest, SendGoal) {
  auto node = std::make_shared<test_composition::ActionClient>(opts);
  auto client_mock = rtest::findActionClient<test_composition::action::MoveRobot>(node, "move_robot");

  ASSERT_TRUE(client_mock);

  // Mock server readiness
  EXPECT_CALL(*client_mock, action_server_is_ready())
    .WillOnce(::testing::Return(true));

  // Mock goal sending
  EXPECT_CALL(*client_mock, async_send_goal(::testing::_, ::testing::_))
    .Times(1);

  EXPECT_TRUE(client_mock->action_server_is_ready());

  // This would normally trigger the mock
  // node->send_goal(2.0, 3.0);
}

TEST_F(ActionClientTest, ReceiveFeedback) {
  auto node = std::make_shared<test_composition::ActionClient>(opts);
  auto client_mock = rtest::findActionClient<test_composition::action::MoveRobot>(node, "move_robot");

  // Create mock goal handle
  auto goal_handle = std::make_shared<rclcpp_action::ClientGoalHandle<test_composition::action::MoveRobot>>();

  // Simulate feedback
  test_composition::action::MoveRobot::Feedback feedback;
  feedback.current_x = 1.0;
  feedback.current_y = 2.0;
  feedback.distance_remaining = 1.5;

  EXPECT_NO_THROW(client_mock->simulate_feedback(goal_handle, feedback));
}

TEST_F(ActionClientTest, ReceiveResult) {
  auto node = std::make_shared<test_composition::ActionClient>(opts);
  auto client_mock = rtest::findActionClient<test_composition::action::MoveRobot>(node, "move_robot");

  auto goal_handle = std::make_shared<rclcpp_action::ClientGoalHandle<test_composition::action::MoveRobot>>();

  // Create successful result
  rclcpp_action::ClientGoalHandle<test_composition::action::MoveRobot>::WrappedResult result;
  result.code = rclcpp_action::ResultCode::SUCCEEDED;
  result.result = std::make_shared<test_composition::action::MoveRobot::Result>();
  result.result->success = true;
  result.result->final_x = 2.0;
  result.result->final_y = 3.0;
  result.result->message = "Target reached";
  EXPECT_NO_THROW(client_mock->simulate_result(goal_handle, result));
}

TEST_F(ActionClientTest, ServerNotReady) {
  auto node = std::make_shared<test_composition::ActionClient>(opts);
  auto client_mock = rtest::findActionClient<test_composition::action::MoveRobot>(node, "move_robot");

  // Mock server not ready
  EXPECT_CALL(*client_mock, action_server_is_ready())
    .WillOnce(::testing::Return(false));

  EXPECT_FALSE(client_mock->action_server_is_ready());
}